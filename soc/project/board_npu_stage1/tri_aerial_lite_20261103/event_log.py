"""Bounded, optional metadata-only disk writer; no inference backpressure.

enqueue success means queued, NOT saved. snapshot distinguishes written bytes,
capacity drops, I/O failure and final fsync. No frame pixels are accepted.
"""
import json
import os
from pathlib import Path
import queue
import threading
import time


class AsyncEventLog:
    def __init__(self, path, *, capacity=128, max_bytes=4*1024*1024):
        if type(capacity) is not int or not 1 <= capacity <= 1024:
            raise ValueError('bounded queue capacity required')
        if type(max_bytes) is not int or not 1024 <= max_bytes <= 64*1024*1024:
            raise ValueError('bounded log byte budget required')
        self.path = Path(path)
        self.capacity, self.max_bytes = capacity, max_bytes
        self.queue = queue.Queue(capacity)
        self.lock = threading.RLock()
        self.stop = threading.Event(); self.ready = threading.Event()
        self.thread = None
        self.error = None
        self.accepted = self.written = self.bytes_written = self.failed = self.inflight = 0
        self.dropped_full = self.dropped_budget = self.rejected = 0
        self.fsynced_records = 0
        self.closed = False

    def start(self, timeout=2):
        with self.lock:
            if self.thread is not None:
                raise RuntimeError('one writer lifecycle required')
            self.thread = threading.Thread(target=self._run, name='tri-observation-log', daemon=False)
            self.thread.start()
        if not self.ready.wait(timeout):
            raise TimeoutError('log file preparation did not finish')
        return self.snapshot()

    def enqueue(self, event):
        # Encoding remains bounded to small metadata and captures caller-owned
        # mutable dictionaries now; the disk worker never borrows image memory.
        if type(event) is not dict or event.get('schema') != 'tri-aerial-event/1':
            raise ValueError('explicit observation event required')
        data = (json.dumps(event, ensure_ascii=False, allow_nan=False, separators=(',', ':'))+'\n').encode('utf8')
        if len(data) > 8192:
            raise ValueError('event exceeds 8 KiB metadata limit')
        with self.lock:
            if self.thread is None or not self.ready.is_set() or self.stop.is_set() or self.closed or self.error:
                self.rejected += 1
                return False
            try:
                self.queue.put_nowait(data)
            except queue.Full:
                self.dropped_full += 1
                return False
            self.accepted += 1
            return True

    def snapshot(self):
        with self.lock:
            alive = bool(self.thread and self.thread.is_alive())
            return dict(schema='tri-aerial-log-state/1', path=str(self.path), capacity=self.capacity,
                max_bytes=self.max_bytes, queued=self.queue.qsize(), inflight=self.inflight,
                accepted=self.accepted, written=self.written, bytes_written=self.bytes_written,
                failed=self.failed, dropped_full=self.dropped_full, dropped_budget=self.dropped_budget,
                rejected=self.rejected, fsynced_records=self.fsynced_records, error=self.error,
                alive=alive, closed=self.closed, clean=self.closed and not alive,
                complete=self.closed and not alive and self.error is None and self.accepted == self.written
                    and self.dropped_full == self.dropped_budget == self.rejected == 0,
                storage='caller-selected local filesystem; not an SD-card or upload claim',
                enqueue_is_not_persistence=True)

    def close(self, timeout=3):
        with self.lock:
            self.stop.set()
        if self.thread is not None:
            self.thread.join(timeout)
        return self.snapshot()

    def _write(self, fd, data):
        while data:
            written = os.write(fd, data)
            if written <= 0:
                raise OSError('log write made no progress')
            data = data[written:]

    def _run(self):
        fd = None
        try:
            # Explicit non-overwrite destination; no directory creation or
            # retention deletion is hidden in the capture-side component.
            fd = os.open(self.path, os.O_WRONLY|os.O_CREAT|os.O_EXCL|getattr(os, 'O_NOFOLLOW', 0), 0o600)
            self.ready.set()
            while True:
                with self.lock:
                    if self.stop.is_set() and self.queue.empty():
                        break
                try:
                    data = self.queue.get(timeout=.05)
                except queue.Empty:
                    continue
                with self.lock:
                    self.inflight = 1
                    budget = self.bytes_written+len(data) <= self.max_bytes
                try:
                    if not budget:
                        with self.lock:
                            self.dropped_budget += 1
                    else:
                        self._write(fd, data)
                        with self.lock:
                            self.written += 1; self.bytes_written += len(data)
                except BaseException:
                    with self.lock:
                        self.failed += 1
                    raise
                finally:
                    with self.lock:
                        self.inflight = 0
                    self.queue.task_done()
            os.fsync(fd)
            with self.lock:
                self.fsynced_records = self.written
        except BaseException as exc:
            with self.lock:
                self.error = repr(exc)
            while True:
                try:
                    self.queue.get_nowait()
                except queue.Empty:
                    break
                with self.lock:
                    self.failed += 1
                self.queue.task_done()
        finally:
            try:
                if fd is not None:
                    os.close(fd)
            except BaseException as exc:
                with self.lock:
                    self.error = repr(exc)
            with self.lock:
                self.closed = True
            self.ready.set()
