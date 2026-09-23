#!/usr/bin/env bash
set -euo pipefail
# Self-contained launcher: Python body is embedded so this .sh alone is portable.
export PYTHONDONTWRITEBYTECODE=1
python3 - "$@" <<'PY'
import argparse, os, uuid, json, math
from pathlib import Path
import numpy as np
import yaml
from scipy.ndimage import gaussian_filter1d
from scipy.spatial.transform import Rotation
p = argparse.ArgumentParser(description='Trajectory CSV -> schemaVersion 2 camera YAML. Positions use original coordinates; camera faces smoothed travel direction with world Z up.')
p.add_argument('--input', type=Path, required=True, help='CSV with timestamp,x,y,z,qx,qy,qz,qw')
p.add_argument('--output', type=Path, default=Path('Camera-path.yaml'))
p.add_argument('--duration', type=float, help='Playback duration in seconds; defaults to source elapsed time')
p.add_argument('--interval', type=float, default=10.0, help='Maximum waypoint interval in seconds (uniform intervals including both endpoints)')
p.add_argument('--name', default='Measured trajectory')
p.add_argument('--look-distance', type=float, default=10.0)
p.add_argument('--heading-window', type=float, default=2.5, help='Seconds on either side for direction estimation')
p.add_argument('--smoothing', type=float, default=1.0, help='Heading smoothing in seconds')
p.add_argument('--stationary-speed', type=float, default=0.1, help='Below this m/s, interpolate heading between moving portions')
p.add_argument('--max-pitch', type=float, default=10.0, help='Maximum camera pitch magnitude in degrees')
p.add_argument('--height-offset', type=float, default=0.0, help='Additional camera height; positions are already sensor-height coordinates')
p.add_argument('--fov', type=float, default=60.0)
p.add_argument('--force', action='store_true', help='Explicitly replace existing YAML and timing sidecar')
a = p.parse_args()
if a.duration is not None and (not np.isfinite(a.duration) or a.duration <= 0):
    p.error('duration must be finite and positive')
for k in ['interval', 'look_distance', 'heading_window']:
    if not np.isfinite(getattr(a, k)) or getattr(a, k) <= 0:
        p.error(k + ' must be finite and positive')
if not 0 < a.fov < 180 or not 0 <= a.max_pitch < 89 or (not np.isfinite(a.height_offset)) or (not np.isfinite(a.smoothing)) or (a.smoothing < 0) or (not np.isfinite(a.stationary_speed)) or (a.stationary_speed < 0):
    p.error('Invalid camera/smoothing settings')
c = np.genfromtxt(a.input, delimiter=',', names=True)
if c.ndim != 1 or len(c) < 2:
    p.error('Need at least two trajectory rows')
if not {'timestamp', 'x', 'y', 'z'}.issubset(c.dtype.names):
    p.error('Missing timestamp/x/y/z column')
t = c['timestamp']
xyz = np.column_stack([c[k] for k in ['x', 'y', 'z']])
if not np.isfinite(t).all() or not np.isfinite(xyz).all() or (not (np.diff(t) > 0).all()):
    p.error('Timestamps must increase strictly; all time/XYZ values must be finite')
t = t - t[0]
duration = float(t[-1])
n = int(math.ceil(duration / a.interval)) + 1
times = np.linspace(0, duration, n)

def interp(ts):
    return np.column_stack([np.interp(ts, t, xyz[:, i]) for i in range(3)])
grid = np.linspace(0, duration, max(2, int(math.ceil(duration / 0.1)) + 1))
dt = grid[1] - grid[0]
left = np.maximum(0, grid - a.heading_window)
right = np.minimum(duration, grid + a.heading_window)
v = (interp(right) - interp(left)) / (right - left)[:, None]
speed = np.linalg.norm(v[:, :2], axis=1)
moving = speed >= a.stationary_speed
if not moving.any():
    p.error('No horizontal motion detected; cannot infer a travel heading')
yaw = np.interp(grid, grid[moving], np.unwrap(np.arctan2(v[moving, 1], v[moving, 0])))
pitch = np.interp(grid, grid[moving], np.arctan2(v[moving, 2], speed[moving]))
if a.smoothing > 0:
    yaw = gaussian_filter1d(yaw, a.smoothing / dt, mode='nearest')
    pitch = gaussian_filter1d(pitch, a.smoothing / dt, mode='nearest')
yaw = np.interp(times, grid, yaw)
pitch = np.clip(np.interp(times, grid, pitch), -np.deg2rad(a.max_pitch), np.deg2rad(a.max_pitch))
positions = interp(times)
positions[:, 2] += a.height_offset
forward = np.column_stack([np.cos(pitch) * np.cos(yaw), np.cos(pitch) * np.sin(yaw), np.sin(pitch)])
waypoints = []
previous = None
for (pos, f) in zip(positions, forward):
    right = np.cross(f, [0.0, 0.0, 1.0])
    right /= np.linalg.norm(right)
    up = np.cross(right, f)
    q = Rotation.from_matrix(np.column_stack([right, up, -f])).as_quat()
    if previous is not None and np.dot(previous, q) < 0:
        q = -q
    previous = q
    waypoints.append(dict(id=str(uuid.uuid4()), position=pos.tolist(), target=(pos + a.look_distance * f).tolist(), quaternion=q.tolist(), fov=a.fov))
result = dict(schemaVersion=2, name=a.name, durationSeconds=duration if a.duration is None else a.duration, waypoints=waypoints)
sidecar = a.output.with_suffix('.timing.csv')
if not a.force and (a.output.exists() or sidecar.exists()):
    p.error('Output already exists; choose another path or explicitly use --force')
a.output.parent.mkdir(parents=True, exist_ok=True)
text = yaml.safe_dump(result, sort_keys=False, allow_unicode=True)
check = yaml.safe_load(text)
assert len(set((w['id'] for w in check['waypoints']))) == n
for w in check['waypoints']:
    q = np.array(w['quaternion'])
    f = np.array(w['target']) - w['position']
    f /= np.linalg.norm(f)
    assert abs(np.linalg.norm(q) - 1) < 1e-12
    assert np.allclose(Rotation.from_quat(q).apply([0, 0, -1]), f, atol=1e-10)
    assert Rotation.from_quat(q).apply([0, 1, 0])[2] > 0
assert np.allclose(positions[[0, -1]], xyz[[0, -1]] + [0, 0, a.height_offset])
assert np.max(np.diff(times)) <= a.interval + 1e-09
if any((np.dot(waypoints[i]['quaternion'], waypoints[i + 1]['quaternion']) < 0 for i in range(n - 1))):
    raise RuntimeError('Quaternion hemisphere discontinuity')
a.output.write_text(text, encoding='utf-8')
np.savetxt(sidecar, np.column_stack([times, times + c['timestamp'][0], positions]), delimiter=',', header='elapsed_seconds,source_timestamp,x,y,z', comments='')
print(json.dumps(dict(output=str(a.output), input=str(a.input), waypoints=n, durationSeconds=result['durationSeconds'], sourceDurationSeconds=duration, actualIntervalSeconds=duration / (n - 1), quaternionOrder='xyzw', cameraForward='-Z', worldUp='+Z', maxPitchDegrees=float(np.max(abs(np.rad2deg(pitch)))), heading='smoothed travel direction, roll suppressed', timing=str(sidecar)), ensure_ascii=False, indent=2))
PY
