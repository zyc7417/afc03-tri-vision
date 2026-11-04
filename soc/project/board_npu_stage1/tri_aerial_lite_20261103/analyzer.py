"""Bounded, per-camera IoU tracking of exact completed detector metadata.

Coordinates are each camera's oriented 1920x1080 source image, NOT ground or
geographic coordinates. IDs identify association episodes, not unique people.
No image copy, motion compensation, extra model call or frame interpolation.
"""
from collections import Counter
from dataclasses import dataclass
import copy
import math
import time

PORTS = ('A0', 'A1', 'B0')
SELECTED = {0: 'person', 1: 'bicycle', 2: 'car', 3: 'motorcycle', 5: 'bus', 7: 'truck'}
TOKEN_KEYS = ('epoch', 'frame_id', 'slot', 'generation')


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def token_tuple(value):
    if type(value) is not dict or set(value) != set(TOKEN_KEYS):
        raise ValueError('exact four-field descriptor required')
    if any(type(value[k]) is not int or value[k] < 0 for k in TOKEN_KEYS):
        raise ValueError('nonnegative integer descriptor required')
    if value['slot'] not in (0, 1) or not 1 <= value['generation'] <= 0xffffffff or any(value[k] > 0xffffffff for k in ('epoch', 'frame_id')):
        raise ValueError('descriptor outside the actual two-slot uint32 contract')
    return tuple(value[k] for k in TOKEN_KEYS)


def valid_box(value):
    if type(value) not in (list, tuple) or len(value) != 4 or not all(finite(v) for v in value):
        raise ValueError('finite four-coordinate box required')
    x1, y1, x2, y2 = map(float, value)
    if not 0 <= x1 < x2 <= 1920 or not 0 <= y1 < y2 <= 1080:
        raise ValueError('box must lie in oriented source 1920x1080 coordinates')
    return x1, y1, x2, y2


@dataclass(frozen=True)
class Policy:
    max_tracks_per_camera: int = 32
    iou_threshold: float = .25
    max_missing_s: float = .4
    reset_gap_s: float = .75
    confirm_hits: int = 3
    # Tuple of (port, x1,y1,x2,y2), normalized to the oriented source image.
    # Empty default deliberately does not invent a scene-specific alarm area.
    screen_rois: tuple = ()
    notice_s: float = 2.
    warning_s: float = 5.

    def __post_init__(self):
        if type(self.max_tracks_per_camera) is not int or not 1 <= self.max_tracks_per_camera <= 64:
            raise ValueError('1..64 tracks per camera required')
        if type(self.confirm_hits) is not int or not 1 <= self.confirm_hits <= 30:
            raise ValueError('1..30 confirmation hits required')
        for name in ('iou_threshold', 'max_missing_s', 'reset_gap_s', 'notice_s', 'warning_s'):
            if not finite(getattr(self, name)):
                raise ValueError('finite policy values required')
        if not 0 < self.iou_threshold <= 1 or not 0 < self.max_missing_s <= self.reset_gap_s <= 10:
            raise ValueError('invalid association/gap limits')
        if not 0 < self.notice_s < self.warning_s <= 3600:
            raise ValueError('ordered screen dwell thresholds required')
        if type(self.screen_rois) is not tuple or len(self.screen_rois) > 3:
            raise ValueError('bounded immutable ROI tuple required')
        seen = set()
        for item in self.screen_rois:
            if type(item) is not tuple or len(item) != 5 or item[0] not in PORTS or item[0] in seen:
                raise ValueError('one normalized ROI per actual camera')
            p, x1, y1, x2, y2 = item
            if not all(finite(v) for v in item[1:]) or not 0 <= x1 < x2 <= 1 or not 0 <= y1 < y2 <= 1:
                raise ValueError('normalized screen ROI required')
            seen.add(p)


def iou(a, b):
    x = min(a[2], b[2]) - max(a[0], b[0])
    y = min(a[3], b[3]) - max(a[1], b[1])
    if x <= 0 or y <= 0:
        return 0.
    intersection = x*y
    return intersection/((a[2]-a[0])*(a[3]-a[1]) + (b[2]-b[0])*(b[3]-b[1]) - intersection)


class LiteAnalyzer:
    """One instance per actual RUN, kept across its finite capture batches.

    update_once is called in ordered render completion, never capture admission.
    An immediately repeated exact descriptor returns cached results without
    advancing tracking or generating events. Older/reordered tokens fail closed.
    Caller must not share one instance across unrelated RUN IDs.
    """
    def __init__(self, run_id, policy=None):
        if type(run_id) is not str or not 1 <= len(run_id) <= 100:
            raise ValueError('bounded nonempty run identity required')
        self.run_id = run_id
        self.policy = policy or Policy()
        if type(self.policy) is not Policy:
            raise TypeError('explicit Policy required')
        self.tracks = [dict() for _ in PORTS]
        self.next_id = [1, 1, 1]
        self.confirmed_episodes = [Counter() for _ in PORTS]
        self.rois = {x[0]: x[1:] for x in self.policy.screen_rois}
        self.last_token = None
        self.last_time = None
        self.last_group = None
        self.last_sources = None
        self.cached = None
        self.frames = self.duplicates = self.restarts = self.limited = 0
        self.total_ns = self.max_ns = 0

    def snapshot(self):
        return dict(schema='tri-aerial-lite-state/1', run_id=self.run_id,
            frames=self.frames, duplicate_calls=self.duplicates,
            long_gap_resets=self.restarts, detections_not_tracked_due_to_capacity=self.limited,
            retained_tracks=[len(x) for x in self.tracks], max_tracks_per_camera=self.policy.max_tracks_per_camera,
            update_mean_ms=self.total_ns/max(1, self.frames)/1e6, update_max_ms=self.max_ns/1e6,
            ROI_enabled=bool(self.rois), coordinates='oriented per-camera 1920x1080 source pixels',
            IDs_are_association_episodes=True, reidentification=False,
            camera_motion_compensated=False, geographic_intrusion=False)

    def _validate(self, tri, descriptor, observed_s):
        token = token_tuple(descriptor)
        if type(tri) is not dict or token_tuple(tri.get('descriptor')) != token:
            raise ValueError('tri metadata must match exact inference descriptor')
        if not finite(observed_s) or observed_s < 0:
            raise ValueError('real monotonic observation time required')
        if tri.get('epoch') != token[0] or type(tri.get('group_id')) is not int or tri['group_id'] < 0:
            raise ValueError('tri epoch/group identity required')
        if tri.get('active_mask') != 7 or tri.get('active_channels') != 3:
            raise ValueError('three active cameras A0 A1 B0 required')
        tiles = tri.get('tiles')
        if type(tiles) is not list or len(tiles) != 4:
            raise ValueError('exact four mosaic tiles required')
        sources = []
        for s, tile in enumerate(tiles):
            if type(tile) is not dict or tile.get('stream_id') != s or tile.get('port') != (*PORTS, 'B1')[s] or tile.get('enabled') is not (s < 3):
                raise ValueError('tile/source identity mismatch')
            if s < 3:
                seq = tile.get('source_seq')
                if type(seq) is not int or seq < 0:
                    raise ValueError('actual source sequence required')
                sources.append(seq)
        rows, indices = tri.get('detections_in_source_coordinates'), tri.get('display_detection_indices')
        if type(rows) is not list or type(indices) is not list or len(rows) != len(indices) or len(rows) > 1000:
            raise ValueError('bounded aligned source detection/index lists required')
        if any(type(i) is not int or i < 0 for i in indices) or len(set(indices)) != len(indices):
            raise ValueError('unique original detection indices required')
        out = [[], [], []]
        source_ids = {}
        for row, index in zip(rows, indices):
            if type(row) is not dict:
                raise ValueError('detection object required')
            s = row.get('stream_id')
            if type(s) is not int or not 0 <= s < 3 or row.get('port') != PORTS[s] or row.get('included_in_count') is not True:
                raise ValueError('actual enabled camera identity required')
            if row.get('group_id') != tri['group_id'] or type(row.get('source_frame_id')) is not int or row['source_frame_id'] < 0:
                raise ValueError('same-group source identity required')
            if row['source_frame_id'] != sources[s]:
                raise ValueError('detection and source tile frame mismatch')
            if s in source_ids and source_ids[s] != row['source_frame_id']:
                raise ValueError('one source frame per camera/group required')
            source_ids[s] = row['source_frame_id']
            cid, confidence, name = row.get('class_id'), row.get('confidence'), row.get('class_name')
            if type(cid) is not int or cid < 0 or type(name) is not str or not name or len(name) > 80:
                raise ValueError('valid detector class required')
            if not finite(confidence) or not 0 <= confidence <= 1:
                raise ValueError('valid detector confidence required')
            b = valid_box(row.get('bbox_xyxy'))
            if cid not in SELECTED:
                continue
            if name != SELECTED[cid]:
                raise ValueError('selected class ID/name binding differs from COCO model')
            out[s].append(dict(box=b, class_id=cid, class_name=name, confidence=confidence,
                detection_index=index, source_frame_id=row['source_frame_id']))
        return token, tuple(sources), out

    def update_once(self, tri, descriptor=None, observed_s=None):
        began = time.perf_counter_ns()
        if descriptor is None:
            descriptor = tri.get('descriptor') if type(tri) is dict else None
        if observed_s is None:
            observed_s = time.monotonic()
        token, sources, rows = self._validate(tri, descriptor, observed_s)
        if token == self.last_token:
            # Exact rows are bound too: a second incompatible result is not a
            # duplicate that may silently inherit an earlier track assignment.
            signature = tuple(tuple((r['box'], r['class_id'], r['confidence'], r['detection_index'], r['source_frame_id']) for r in group) for group in rows)
            if signature != self.last_signature or tri['group_id'] != self.last_group or sources != self.last_sources:
                raise ValueError('same descriptor carries changed detector metadata')
            self.duplicates += 1
            value = copy.deepcopy(self.cached)
            value.update(duplicate_input=True, events=[])
            return value
        if self.last_token is not None and (token[0] != self.last_token[0] or token[1] <= self.last_token[1]
                or observed_s <= self.last_time or tri['group_id'] <= self.last_group
                or any(a <= b for a, b in zip(sources, self.last_sources))):
            raise ValueError('ordered same-epoch fresh descriptor/time/group required')
        events = []
        def event(kind, **values):
            events.append(dict(schema='tri-aerial-event/1', run_id=self.run_id,
                event=kind, observed_monotonic_s=observed_s, descriptor=dict(descriptor),
                group_id=tri['group_id'], coordinate_scope='screen_source_pixels_not_geographic', **values))
        if self.last_time is not None and observed_s-self.last_time > self.policy.reset_gap_s:
            self.tracks = [dict() for _ in PORTS]
            self.restarts += 1
            event('association_reset', gap_s=observed_s-self.last_time)
        outputs = []
        counts = []
        for s, detections in enumerate(rows):
            tracks = self.tracks[s]
            for identity in list(tracks):
                if observed_s-tracks[identity]['seen'] > self.policy.max_missing_s:
                    del tracks[identity]
            cap = self.policy.max_tracks_per_camera
            limited = max(0, len(detections)-cap)
            self.limited += limited
            current_classes = Counter(d['class_name'] for d in detections)
            detections = sorted(detections, key=lambda x: (-x['confidence'], x['detection_index']))[:cap]
            pairs = []
            for identity, t in tracks.items():
                for n, d in enumerate(detections):
                    if t['class_id'] == d['class_id']:
                        overlap = iou(t['box'], d['box'])
                        if overlap >= self.policy.iou_threshold:
                            pairs.append((-overlap, identity, n))
            used_t, used_d, matched = set(), set(), {}
            for _, identity, n in sorted(pairs):
                if identity not in used_t and n not in used_d:
                    used_t.add(identity); used_d.add(n); matched[n] = identity
            # Missing frames cannot accrue screen-dwell time. Their track IDs
            # may survive a short gap, but their alert timer must start anew.
            for identity, t in tracks.items():
                if identity not in used_t:
                    if t['level']:
                        event('screen_roi_clear', port=PORTS[s], stream_id=s,
                            track_id=PORTS[s]+':'+str(identity), previous_level=t['level'], reason='not_detected_this_frame')
                    t.update(dwell_start=None, level=0, streak=0)
            confirmed_current = Counter()
            for n, d in enumerate(detections):
                identity = matched.get(n)
                if identity is None:
                    if len(tracks) >= cap:
                        # Evict only an unmatched old track. Never discard the
                        # current frame's existing association for a new box.
                        candidates = [(t['seen'], ident) for ident, t in tracks.items() if ident not in used_t]
                        if not candidates:
                            self.limited += 1
                            limited += 1
                            continue
                        del tracks[min(candidates)[1]]
                    identity = self.next_id[s]; self.next_id[s] += 1
                    tracks[identity] = dict(class_id=d['class_id'], box=d['box'], seen=observed_s,
                        hits=0, streak=0, confirmed=False, dwell_start=None, level=0)
                    used_t.add(identity)
                t = tracks[identity]
                t.update(box=d['box'], seen=observed_s, hits=t['hits']+1, streak=t['streak']+1)
                tid = PORTS[s]+':'+str(identity)
                if not t['confirmed'] and t['streak'] >= self.policy.confirm_hits:
                    t['confirmed'] = True
                    self.confirmed_episodes[s][d['class_name']] += 1
                    event('track_confirmed', port=PORTS[s], stream_id=s, track_id=tid, class_name=d['class_name'], bbox_xyxy=list(d['box']))
                if t['confirmed']:
                    confirmed_current[d['class_name']] += 1
                roi = self.rois.get(PORTS[s])
                inside = bool(roi and roi[0] <= (d['box'][0]+d['box'][2])/3840 <= roi[2]
                    and roi[1] <= (d['box'][1]+d['box'][3])/2160 <= roi[3])
                if not inside or not t['confirmed']:
                    if t['level']:
                        event('screen_roi_clear', port=PORTS[s], stream_id=s, track_id=tid, class_name=d['class_name'], previous_level=t['level'], reason='outside_screen_roi')
                    t.update(dwell_start=None, level=0)
                else:
                    if t['dwell_start'] is None:
                        t['dwell_start'] = observed_s
                    dwell = observed_s-t['dwell_start']
                    level = 2 if dwell >= self.policy.warning_s else 1 if dwell >= self.policy.notice_s else 0
                    if level > t['level']:
                        event('screen_roi_dwell', port=PORTS[s], stream_id=s, track_id=tid, class_name=d['class_name'],
                            level=level, level_name='warning' if level == 2 else 'notice', dwell_s=dwell, bbox_xyxy=list(d['box']))
                    t['level'] = level
                outputs.append(dict(port=PORTS[s], stream_id=s, track_id=tid, class_id=d['class_id'], class_name=d['class_name'],
                    confidence=d['confidence'], detection_index=d['detection_index'], source_frame_id=d['source_frame_id'],
                    bbox_xyxy=list(d['box']), confirmed=t['confirmed'], hits=t['hits'],
                    screen_roi_inside=inside, screen_roi_level=t['level'],
                    screen_roi_dwell_s=observed_s-t['dwell_start'] if t['dwell_start'] is not None else 0.))
            counts.append(dict(port=PORTS[s], stream_id=s, current_classes=dict(current_classes),
                people=current_classes['person'], vehicles=sum(v for k, v in current_classes.items() if k != 'person'),
                current_confirmed_classes=dict(confirmed_current),
                confirmed_association_episodes=dict(self.confirmed_episodes[s]),
                detections_limited_this_frame=limited, retained_tracks=len(tracks)))
        value = dict(schema='tri-aerial-lite-result/1', run_id=self.run_id, descriptor=dict(descriptor),
            group_id=tri['group_id'], observed_monotonic_s=observed_s, duplicate_input=False,
            tracks=sorted(outputs, key=lambda x: x['detection_index']), per_camera=counts, events=events,
            current_people=sum(x['people'] for x in counts), current_vehicles=sum(x['vehicles'] for x in counts),
            ROI_enabled=bool(self.rois), current_alert_level=max((x['screen_roi_level'] for x in outputs), default=0),
            count_scope='selected current detections; cumulative values are confirmed association episodes, not unique people',
            coordinates='oriented per-camera source pixels; no geographic calibration or camera motion compensation')
        self.last_signature = tuple(tuple((r['box'], r['class_id'], r['confidence'], r['detection_index'], r['source_frame_id']) for r in group) for group in rows)
        self.last_token, self.last_time, self.last_group, self.last_sources = token, observed_s, tri['group_id'], sources
        self.frames += 1
        self.cached = value
        output = copy.deepcopy(value)
        elapsed = time.perf_counter_ns()-began
        self.total_ns += elapsed; self.max_ns = max(self.max_ns, elapsed)
        return output

    update = update_once
