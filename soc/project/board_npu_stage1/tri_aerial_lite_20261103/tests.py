"""CPU-only contracts and timing. Not camera/NPU/board performance evidence."""
from pathlib import Path
import copy
import json
import platform
import statistics
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from board_npu_stage1.tri_aerial_lite_20261103 import LiteAnalyzer, Policy, AsyncEventLog


def fixture(frame, items=()):
    """items=(port,class_id,box), synthetic completed detector metadata."""
    names = {0:'person', 1:'bicycle', 2:'car', 3:'motorcycle', 5:'bus', 7:'truck', 39:'bottle'}
    ports = ('A0','A1','B0','B1')
    descriptor = dict(epoch=42, frame_id=frame, slot=frame % 2, generation=frame//2+1)
    rows = [dict(stream_id=ports.index(p), port=p, source_frame_id=frame+100*ports.index(p),
        group_id=frame, class_id=c, class_name=names[c], confidence=.9, bbox_xyxy=list(b), included_in_count=True)
        for p,c,b in items]
    tiles = [dict(stream_id=s,port=p,enabled=s<3,source_seq=frame+100*s if s<3 else None,
        display_rect=[0,0,960,540]) for s,p in enumerate(ports)]
    return dict(descriptor=descriptor, epoch=42,group_id=frame,active_mask=7,active_channels=3,
        tiles=tiles,display_detection_indices=list(range(len(rows))),detections_in_source_coordinates=rows)


PERSON = ('A0',0,(100,100,200,350))


class AnalyzerTests(unittest.TestCase):
    def test_three_ports_no_cross_association_and_counts(self):
        engine = LiteAnalyzer('run')
        items=[PERSON,('A1',0,(100,100,200,350)),('B0',2,(400,400,600,650)),('A0',39,(800,100,900,200))]
        for f in range(1,5):
            value=engine.update_once(fixture(f,items),observed_s=f*.02)
        self.assertEqual([t['track_id'] for t in value['tracks']],['A0:1','A1:1','B0:1'])
        self.assertEqual((value['current_people'],value['current_vehicles']),(2,1))
        self.assertTrue(all(t['confirmed'] for t in value['tracks']))
        self.assertEqual(sum(sum(p['confirmed_association_episodes'].values()) for p in value['per_camera']),3)
        self.assertFalse(value['ROI_enabled'])

    def test_short_occlusion_reuses_id_long_gap_creates_new_episode(self):
        engine=LiteAnalyzer('run',Policy(confirm_hits=1))
        a=engine.update_once(fixture(1,[PERSON]),observed_s=1.)
        engine.update_once(fixture(2),observed_s=1.1)
        b=engine.update_once(fixture(3,[PERSON]),observed_s=1.2)
        c=engine.update_once(fixture(4,[PERSON]),observed_s=2.2)
        self.assertEqual(a['tracks'][0]['track_id'],b['tracks'][0]['track_id'])
        self.assertNotEqual(b['tracks'][0]['track_id'],c['tracks'][0]['track_id'])
        self.assertIn('association_reset',[e['event'] for e in c['events']])
        self.assertEqual(c['per_camera'][0]['confirmed_association_episodes']['person'],2)

    def test_duplicate_cache_no_events_or_advance_and_mutation_isolated(self):
        engine=LiteAnalyzer('run',Policy(confirm_hits=1))
        tri=fixture(1,[PERSON]);a=engine.update_once(tri,observed_s=1.)
        a['tracks'][0]['bbox_xyxy'][0]=999
        b=engine.update_once(tri,observed_s=2.)
        self.assertEqual(engine.frames,1)
        self.assertTrue(b['duplicate_input']);self.assertEqual(b['events'],[])
        self.assertEqual(b['tracks'][0]['bbox_xyxy'][0],100.)
        changed=copy.deepcopy(tri);changed['detections_in_source_coordinates'][0]['bbox_xyxy'][0]=101
        with self.assertRaises(ValueError):engine.update_once(changed,observed_s=2.)

    def test_stale_and_invalid_inputs_preserve_state(self):
        engine=LiteAnalyzer('run');engine.update_once(fixture(2,[PERSON]),observed_s=2.)
        cases=[]
        for mutate in [lambda x:x['descriptor'].update(epoch=99),
                       lambda x:x['detections_in_source_coordinates'][0].update(confidence=float('nan')),
                       lambda x:x['detections_in_source_coordinates'][0].update(bbox_xyxy=[0,0,float('inf'),1]),
                       lambda x:x['detections_in_source_coordinates'][0].update(port='B1'),
                       lambda x:x['tiles'][0].update(source_seq=2),
                       lambda x:x['detections_in_source_coordinates'][0].update(class_name='truck')]:
            tri=fixture(3,[PERSON]);mutate(tri);cases.append(tri)
        cases.append(fixture(1,[PERSON]))
        before=engine.snapshot()
        for tri in cases:
            with self.assertRaises(ValueError):engine.update_once(tri,observed_s=3.)
        self.assertEqual(before,engine.snapshot())
        with self.assertRaises(ValueError):engine.update_once(fixture(3),observed_s=2.)

    def test_explicit_roi_levels_and_missing_frame_resets_dwell(self):
        policy=Policy(confirm_hits=1,screen_rois=(('A0',0.,0.,.5,.5),),notice_s=.2,warning_s=.5)
        engine=LiteAnalyzer('run',policy);events=[]
        for frame in range(1,8):
            v=engine.update_once(fixture(frame,[PERSON]),observed_s=1.+(frame-1)*.1)
            events.extend(v['events'])
        levels=[e['level'] for e in events if e['event']=='screen_roi_dwell']
        self.assertEqual(levels,[1,2]);self.assertEqual(v['current_alert_level'],2)
        missing=engine.update_once(fixture(8),observed_s=1.7)
        self.assertEqual([e['event'] for e in missing['events']],['screen_roi_clear'])
        self.assertEqual(missing['events'][0]['reason'],'not_detected_this_frame')
        again=engine.update_once(fixture(9,[PERSON]),observed_s=1.8)
        self.assertEqual(again['tracks'][0]['screen_roi_dwell_s'],0)
        self.assertEqual(again['current_alert_level'],0)
        moved=engine.update_once(fixture(10,[('A0',0,(1500,800,1650,1000))]),observed_s=1.9)
        self.assertFalse(moved['tracks'][0]['screen_roi_inside'])

    def test_capacity_limits_tracking_but_keeps_current_detection_count(self):
        engine=LiteAnalyzer('run',Policy(max_tracks_per_camera=2))
        items=[('A0',0,(100+i*100,100,150+i*100,250)) for i in range(5)]
        v=engine.update_once(fixture(1,items),observed_s=1)
        self.assertEqual(v['current_people'],5)
        self.assertEqual(len(v['tracks']),2)
        self.assertEqual(v['per_camera'][0]['detections_limited_this_frame'],3)
        self.assertEqual(engine.snapshot()['retained_tracks'],[2,0,0])

    def test_new_run_is_fresh_and_per_camera_class_change_does_not_match(self):
        e=LiteAnalyzer('run1',Policy(confirm_hits=1))
        e.update_once(fixture(1,[PERSON]),observed_s=1)
        v=e.update_once(fixture(2,[('A0',2,PERSON[2])]),observed_s=1.1)
        self.assertEqual(v['tracks'][0]['track_id'],'A0:2')
        fresh=LiteAnalyzer('run2').update_once(fixture(1,[PERSON]),observed_s=100.)
        self.assertEqual(fresh['tracks'][0]['track_id'],'A0:1')
        self.assertEqual(fresh['run_id'],'run2')


def event(i):
    return dict(schema='tri-aerial-event/1',run_id='synthetic',event='test',number=i,scope='CPU fixture')


class LogTests(unittest.TestCase):
    def test_log_persists_exact_records_and_no_overwrite(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'events.jsonl';log=AsyncEventLog(path);self.assertIsNone(log.start()['error'])
            for i in range(10):self.assertTrue(log.enqueue(event(i)))
            state=log.close();self.assertTrue(state['complete']);self.assertEqual(state['fsynced_records'],10)
            self.assertEqual([json.loads(x)['number'] for x in path.read_text().splitlines()],list(range(10)))
            existing=path.read_bytes();duplicate=AsyncEventLog(path);self.assertIsNotNone(duplicate.start()['error'])
            self.assertFalse(duplicate.enqueue(event(11)));self.assertTrue(duplicate.close()['clean'])
            self.assertEqual(path.read_bytes(),existing)

    def test_queue_overflow_and_timed_close_do_not_block_capture(self):
        class Blocked(AsyncEventLog):
            def _write(self,fd,data):
                entered.set();release.wait(3);super()._write(fd,data)
        entered=threading.Event();release=threading.Event()
        with tempfile.TemporaryDirectory() as folder:
            log=Blocked(Path(folder)/'x',capacity=1);log.start()
            try:
                self.assertTrue(log.enqueue(event(1)));self.assertTrue(entered.wait(1))
                self.assertTrue(log.enqueue(event(2)))
                begin=time.perf_counter();self.assertFalse(log.enqueue(event(3)))
                self.assertLess(time.perf_counter()-begin,.1)
                s=log.close(.01);self.assertFalse(s['clean']);self.assertTrue(s['alive'])
            finally:
                release.set();s=log.close(2)
            self.assertTrue(s['clean']);self.assertEqual(s['written'],2);self.assertEqual(s['dropped_full'],1)
            self.assertFalse(s['complete'])

    def test_write_failure_visible_no_fake_saved_count(self):
        class Broken(AsyncEventLog):
            def _write(self,fd,data):raise OSError('injected disk failure')
        with tempfile.TemporaryDirectory() as folder:
            log=Broken(Path(folder)/'x');log.start();log.enqueue(event(1));s=log.close(2)
            self.assertTrue(s['clean']);self.assertIn('injected disk failure',s['error'])
            self.assertEqual(s['written'],0);self.assertEqual(s['failed'],1);self.assertFalse(s['complete'])

    def test_disk_budget_bounded_and_bad_event_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'x';log=AsyncEventLog(path,capacity=128,max_bytes=1024);log.start()
            with self.assertRaises(ValueError):log.enqueue(dict(schema='tri-aerial-event/1',data=float('nan')))
            for i in range(30):log.enqueue(event(i))
            s=log.close();self.assertGreater(s['dropped_budget'],0)
            self.assertLessEqual(path.stat().st_size,1024)
            self.assertEqual(s['accepted'],s['written']+s['failed']+s['dropped_budget'])

    def test_fsync_failure_is_not_durable_success(self):
        with tempfile.TemporaryDirectory() as folder:
            log=AsyncEventLog(Path(folder)/'x');log.start();log.enqueue(event(1))
            with patch('board_npu_stage1.tri_aerial_lite_20261103.event_log.os.fsync',side_effect=OSError('injected fsync')):
                s=log.close()
            self.assertEqual(s['written'],1);self.assertEqual(s['fsynced_records'],0)
            self.assertFalse(s['complete']);self.assertIsNotNone(s['error'])


def benchmark():
    out=[]
    for per_camera in (0,1,8,32,64):
        e=LiteAnalyzer('cpu-benchmark',Policy(max_tracks_per_camera=64))
        times=[]
        for f in range(1,202):
            items=[]
            for p in ('A0','A1','B0'):
                for i in range(per_camera):
                    x=20+(i%8)*230+(f%3);y=20+(i//8)*125
                    items.append((p,0 if i%2==0 else 2,(x,y,x+100,y+100)))
            tri=fixture(f,items)
            begin=time.perf_counter_ns();e.update_once(tri,observed_s=f*.02)
            if f>1:times.append((time.perf_counter_ns()-begin)/1e6)
        ordered=sorted(times)
        out.append(dict(detections=3*per_camera,frames=len(times),mean_ms=statistics.mean(times),
            p95_ms=ordered[int(.95*len(ordered))-1],max_ms=max(times),state=e.snapshot()))
    return dict(scope='synthetic detector metadata, Windows/host CPU only; not end-to-end or board FPS',
        python=sys.version,platform=platform.platform(),cases=out)


if __name__=='__main__':
    suite=unittest.defaultTestLoader.loadTestsFromModule(sys.modules[__name__])
    result=unittest.TextTestRunner(verbosity=2).run(suite)
    evidence=dict(passed=result.wasSuccessful(),tests=result.testsRun,
        failures=[str(x) for x in result.failures],errors=[str(x) for x in result.errors],
        remaining_writer_threads=[t.name for t in threading.enumerate() if t.name=='tri-observation-log'])
    if result.wasSuccessful():evidence['benchmark']=benchmark()
    destination=Path(__file__).parent/'cpu_evidence.json'
    destination.write_text(json.dumps(evidence,ensure_ascii=False,indent=2),'utf8')
    print(json.dumps(evidence,ensure_ascii=False,indent=2))
    raise SystemExit(0 if evidence['passed'] and not evidence['remaining_writer_threads'] else 1)
