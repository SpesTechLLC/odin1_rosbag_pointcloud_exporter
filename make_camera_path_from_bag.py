#!/usr/bin/env python3
"""Export a time-windowed camera path using bag record timestamps.

The shell entry point sources ROS. Cached CSV input avoids rereading large bags.
"""
from pathlib import Path
from datetime import datetime, time as dtime
from zoneinfo import ZoneInfo
import argparse
import json
import re
import subprocess
import time
from decimal import Decimal, InvalidOperation
import numpy as np
import yaml
import rosbag2_py as b
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation, Slerp
p = argparse.ArgumentParser(description='ROS bag -> time-windowed camera path. --start/--end accept elapsed seconds, local HH:MM:SS, or ISO datetime. Wall time uses bag record timestamps, not device header timestamps.')
p.add_argument('bag', type=Path)
p.add_argument('--output', required=True, type=Path)
p.add_argument('--start', default='0')
p.add_argument('--end')
p.add_argument('--timezone', default='Asia/Tokyo')
p.add_argument('--interval', type=float, default=10.0)
p.add_argument('--duration', type=float)
p.add_argument('--topic', default='/odin1/odometry')
p.add_argument('--force', action='store_true')
p.add_argument('--cached-csv', type=Path, help='Reuse a previously extracted .source.csv covering the selected boundaries')
a = p.parse_args()
zone = ZoneInfo(a.timezone)
meta = yaml.safe_load((a.bag / 'metadata.yaml').read_text())['rosbag2_bagfile_information']
origin_ns = int(meta['starting_time']['nanoseconds_since_epoch'])
end_ns = origin_ns + int(meta['duration']['nanoseconds'])
origin = origin_ns / 1000000000.0

def bound(s, default):
    if s is None:
        return default
    try:
        return origin_ns + int(Decimal(s) * 1000000000)
    except (InvalidOperation, ValueError, OverflowError):
        pass
    # Python 3.10 accepts only 3 or 6 fractional digits in fromisoformat.
    s = re.sub(r'(\d{2}:\d{2}:\d{2})[.,](\d{1,6})(?=$|[+-])',
               lambda m: m[1] + '.' + m[2].ljust(6, '0'), s)
    if 'T' in s or ' ' in s:
        dt = datetime.fromisoformat(s)
    else:
        dt = datetime.combine(datetime.fromtimestamp(origin, zone).date(), dtime.fromisoformat(s))
    if dt.tzinfo is None:
        dt = dt.replace(tzinfo=zone)
    return int(round(dt.timestamp() * 1000000000.0))
try:
    lo = bound(a.start, origin_ns)
    hi = bound(a.end, end_ns)
except ValueError as exc:
    p.error(f'Invalid start/end time: {exc}')
if not np.isfinite(a.interval) or a.interval <= 0:
    p.error('interval must be finite and positive')
if a.duration is not None and (not np.isfinite(a.duration) or a.duration <= 0):
    p.error('duration must be finite and positive')
if not origin_ns <= lo < hi <= end_ns:
    p.error('Require bag start <= selected start < selected end <= bag end. For another date use ISO datetime.')
a.output.parent.mkdir(parents=True, exist_ok=True)
base = a.output.with_suffix('')
source = Path(str(base) + '.source.csv')
cropped = Path(str(base) + '.trajectory.csv')
info = Path(str(base) + '.range.json')
gen = Path(str(base) + '.generation.json')
for f in [a.output, a.output.with_suffix('.timing.csv'), source, cropped, info, gen]:
    if f.exists() and (not a.force):
        p.error(f'Output exists: {f}; use --force or another output path')
print('Selected', datetime.fromtimestamp(lo / 1000000000.0, zone).isoformat(), 'through', datetime.fromtimestamp(hi / 1000000000.0, zone).isoformat(), flush=True)
rows = []
frames = set()
lastprogress = time.monotonic()
if a.cached_csv:
    previous_info = a.cached_csv.with_name(a.cached_csv.name.removesuffix('.source.csv') + '.range.json')
    if previous_info.exists():
        cached_info = json.loads(previous_info.read_text())
        if Path(cached_info['bag']).resolve() != a.bag.resolve() or cached_info['topic'] != a.topic:
            p.error('Cached trajectory belongs to a different bag or topic')
        frames = {tuple(x) for x in cached_info.get('frames', [])}
    c = np.atleast_1d(np.genfromtxt(a.cached_csv, delimiter=',', names=True))
    rows = np.column_stack([c[k] for k in ['timestamp', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw', 'header_timestamp']]).tolist()
else:
    r = b.SequentialReader()
    r.open(b.StorageOptions(uri=str(a.bag), storage_id=''), b.ConverterOptions('', ''))
    topics = {t.name: t.type for t in r.get_all_topics_and_types()}
    if topics.get(a.topic) != 'nav_msgs/msg/Odometry':
        p.error('Selected topic is absent or not nav_msgs/msg/Odometry')
    r.set_filter(b.StorageFilter(topics=[a.topic]))
    if lo > origin_ns:
        r.seek(max(origin_ns, lo - 2000000000))
    while r.has_next():
        (_, data, ts) = r.read_next()
        m = deserialize_message(data, Odometry)
        q = m.pose.pose.orientation
        v = m.pose.pose.position
        frames.add((m.header.frame_id, m.child_frame_id))
        rows.append([ts / 1000000000.0, v.x, v.y, v.z, q.x, q.y, q.z, q.w, m.header.stamp.sec + m.header.stamp.nanosec * 1e-09])
        if time.monotonic() - lastprogress > 20:
            print(f'Read {len(rows)} odometry messages, current record time {datetime.fromtimestamp(ts / 1000000000.0, zone).isoformat()}', flush=True)
            lastprogress = time.monotonic()
        if ts >= hi:
            break
    del r
    if len(frames) != 1:
        p.error(f'Coordinate frame changes: {frames}')
ar = np.asarray(rows, dtype=float)
if len(ar) < 2 or not np.isfinite(ar).all() or np.any(np.diff(ar[:, 0]) < 0):
    p.error('Invalid / non-monotonic bag record timestamp trajectory')
keep = np.r_[True, np.diff(ar[:, 0]) > 0]
duplicates = int((~keep).sum())
ar = ar[keep]
if len(ar) < 2:
    p.error('Need at least two distinct odometry timestamps')
low = lo / 1000000000.0
high = hi / 1000000000.0
left_extra = max(0.0, ar[0, 0] - low)
right_extra = max(0.0, high - ar[-1, 0])
if left_extra > 0 and lo != origin_ns or (right_extra > 0 and hi != end_ns):
    p.error('Selected interior boundary is not covered by odometry; do not extrapolate')
if max(left_extra, right_extra) > 0.5:
    p.error(f'No nearby odometry at range boundary; extrapolation would be {left_extra}/{right_extra} seconds')
sel = ar[(ar[:, 0] > low) & (ar[:, 0] < high)]
times = np.r_[low, sel[:, 0], high]
coords = np.column_stack([np.interp(times, ar[:, 0], ar[:, i]) for i in [1, 2, 3]])
quats = Slerp(ar[:, 0] - origin, Rotation.from_quat(ar[:, 4:8]))(np.clip(times, ar[0, 0], ar[-1, 0]) - origin).as_quat()
clip = np.column_stack([times, coords, quats])
for (path, values, header) in [(source, ar, 'timestamp,x,y,z,qx,qy,qz,qw,header_timestamp'), (cropped, clip, 'timestamp,x,y,z,qx,qy,qz,qw')]:
    np.savetxt(path, values, delimiter=',', header=header, comments='', fmt='%.12f')
cmd = ['bash', str(Path(__file__).with_name('make_camera_path.sh')), '--input', str(cropped), '--output', str(a.output), '--interval', str(a.interval), '--name', a.bag.name + ' selected trajectory']
cmd += ['--duration', str(a.duration if a.duration is not None else (hi - lo) / 1000000000.0)]
if a.force:
    cmd += ['--force']
run = subprocess.run(cmd, capture_output=True, text=True)
gen.write_text(run.stdout + run.stderr)
if run.returncode:
    raise RuntimeError(run.stdout + run.stderr)
doc = yaml.safe_load(a.output.read_text())
expected = (hi - lo) / 1000000000.0
assert len({w['id'] for w in doc['waypoints']}) == len(doc['waypoints'])
assert np.allclose(doc['waypoints'][0]['position'], clip[0, 1:4]) and np.allclose(doc['waypoints'][-1]['position'], clip[-1, 1:4])
record = dict(bag=str(a.bag), topic=a.topic, time_basis='bag record timestamp, not device header timestamp', timezone=a.timezone, bag_start=datetime.fromtimestamp(origin, zone).isoformat(), start=datetime.fromtimestamp(low, zone).isoformat(), end=datetime.fromtimestamp(high, zone).isoformat(), selected_duration_seconds=expected, durationSeconds=doc['durationSeconds'], waypoints=len(doc['waypoints']), max_interval_seconds=expected / (len(doc['waypoints']) - 1), boundary_hold_seconds=[left_extra, right_extra], duplicate_record_timestamps_removed=duplicates, frames=list(frames), source_csv=str(source), cropped_csv=str(cropped), source_message_count=len(ar), max_record_timestamp_gap_seconds=float(np.diff(ar[:, 0]).max()), command=cmd)
info.write_text(json.dumps(record, ensure_ascii=False, indent=2))
print(json.dumps(record, ensure_ascii=False, indent=2), flush=True)
