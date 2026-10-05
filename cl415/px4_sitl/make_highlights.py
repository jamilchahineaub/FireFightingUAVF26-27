"""highlights from the newest recorded flight: take-off and landing cut from the chase and tail-camera videos,
side by side, plus key frames.

    .venv/Scripts/python px4_sitl/make_highlights.py            (windows, uses the imageio-ffmpeg binary)

the videos run in sim time from the moment the recorders were started; the mission arms about 10 s later.
take-off and touchdown times come from the hull state recorded in the same run (first dry / first wet again),
shifted by the recorder start, which the hull log also marks (its first sample is written when the recorder of
the hull state starts, a few seconds before the video recorders).
"""
import glob
import json
import os
import subprocess
import sys

import imageio_ffmpeg
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
LOGS = os.path.join(HERE, 'flight_logs')
FF = imageio_ffmpeg.get_ffmpeg_exe()


def newest(pattern):
    files = glob.glob(os.path.join(LOGS, pattern))
    return max(files, key=os.path.getmtime) if files else None


def run(args):
    subprocess.run([FF, '-hide_banner', '-loglevel', 'error', '-y'] + args, check=True)


def duration(path):
    out = subprocess.run([FF, '-hide_banner', '-i', path], capture_output=True, text=True).stderr
    for line in out.splitlines():
        if 'Duration' in line:
            h, m, s = line.split('Duration:')[1].split(',')[0].strip().split(':')
            return int(h) * 3600 + int(m) * 60 + float(s)
    return None


def hull_events(path):
    rows = []
    for line in open(path):
        if line.startswith('{'):
            try:
                rows.append(json.loads(line)['data'])
            except Exception:
                pass
    n = max(len(r) for r in rows)
    a = np.array([r for r in rows if len(r) == n])
    a = a[np.argsort(a[:, 0], kind='stable')]
    t, vol, speed = a[:, 0], a[:, 1], a[:, 7]
    moving = np.where(speed > 0.5)[0]
    t_move = t[moving[0]] if len(moving) else t[0]
    dry = np.where((vol < 1e-6) & (t > t_move))[0]
    t_lift = t[dry[0]] if len(dry) else None
    wet = np.where((vol > 1e-6) & (t > (t_lift or t[-1]) + 5))[0]
    t_touch = t[wet[0]] if len(wet) else None
    stop = np.where((speed < 0.5) & (t > (t_touch or t[-1])))[0]
    t_stop = t[stop[0]] if len(stop) else t[-1]
    return t[0], t_move, t_lift, t_touch, t_stop, t[-1]


def main():
    stamp = sys.argv[1] if len(sys.argv) > 1 else None
    chase = newest(f'water_chase_{stamp}*.mp4' if stamp else 'water_chase_*.mp4')
    tail = newest(f'water_tailcam_{stamp}*.mp4' if stamp else 'water_tailcam_*.mp4')
    hull = newest(f'hull_state_water_{stamp}*.jsonl' if stamp else 'hull_state_water_*.jsonl')
    if not chase or not tail:
        print('no recordings found')
        return 1
    tag = os.path.basename(chase).replace('water_chase_', '').replace('.mp4', '')
    dur = duration(chase)
    print(f'chase {os.path.basename(chase)} ({dur:.0f} s), tail {os.path.basename(tail)}, hull {os.path.basename(hull) if hull else None}')
    # the hull recorder starts before the video recorders and stops after them, both in sim time. the chase video
    # is a frozen frame until the take-off run starts, so ffmpeg's freeze detection gives the video time of the
    # first motion, which the hull log also has (speed > 0.5 m/s): that pins the two clocks together
    t0, t_move, t_lift, t_touch, t_stop, t_end = hull_events(hull) if hull else (0, 10, None, None, None, dur)
    fd = subprocess.run([FF, '-hide_banner', '-t', '90', '-i', chase, '-vf', 'freezedetect=n=0.003:d=0.5', '-an', '-f', 'null', '-'],
                        capture_output=True, text=True).stderr
    ends = [float(l.split('freeze_end:')[1].split()[0]) for l in fd.splitlines() if 'freeze_end' in l]
    v_move = ends[0] if ends else 5.0
    offset = (t_move - t0) - v_move        # hull log time (from its first sample) at video time 0
    v = lambda th: max(0.0, th - t0 - offset)
    print(f'first motion in the video at {v_move:.2f} s')
    print(f'hull log: moving {t_move - t0:.1f} s, lift-off {(t_lift - t0) if t_lift else float("nan"):.1f} s, '
          f'touchdown {(t_touch - t0) if t_touch else float("nan"):.1f} s, stopped {t_stop - t0:.1f} s; video time 0 = hull time {offset:.1f} s')
    to_a, to_b = v(t_move) - 3, (v(t_lift) if t_lift else v(t_move) + 20) + 12
    ld_a, ld_b = (v(t_touch) if t_touch else dur - 40) - 15, min(dur, (v(t_stop) if t_touch else dur) + 3)
    segs = [('takeoff', to_a, to_b), ('landing', ld_a, ld_b)]
    parts = []
    for name, a, b in segs:
        out = os.path.join(LOGS, f'water_{name}_{tag}.mp4')
        # chase on the left, tail camera on the right, same height
        run(['-ss', f'{a:.2f}', '-t', f'{b - a:.2f}', '-i', chase, '-ss', f'{a:.2f}', '-t', f'{b - a:.2f}', '-i', tail,
             '-filter_complex', '[0:v]scale=-2:540[l];[1:v]scale=-2:540[r];[l][r]hstack=inputs=2',
             '-c:v', 'libx264', '-pix_fmt', 'yuv420p', '-crf', '20', '-r', '25', out])
        parts.append(out)
        print(f'{name}: video {a:.1f} to {b:.1f} s -> {os.path.basename(out)}')
    lst = os.path.join(LOGS, 'concat.txt')
    open(lst, 'w').write(''.join(f"file '{p}'\n" for p in parts))
    hl = os.path.join(LOGS, f'water_highlights_{tag}.mp4')
    run(['-f', 'concat', '-safe', '0', '-i', lst, '-c', 'copy', hl])
    os.remove(lst)
    # key frames from the chase camera
    frames = []
    for name, tt in (('rest', v(t_move) - 2), ('planing', v(t_lift) - 1.5 if t_lift else v(t_move) + 5),
                     ('liftoff', v(t_lift) + 1.0 if t_lift else v(t_move) + 8), ('climb', v(t_lift) + 6 if t_lift else v(t_move) + 15),
                     ('flare', v(t_touch) - 2 if t_touch else dur - 30), ('touchdown', v(t_touch) + 0.5 if t_touch else dur - 20),
                     ('rollout', v(t_touch) + 5 if t_touch else dur - 10), ('stopped', v(t_stop) if t_touch else dur - 1)):
        png = os.path.join(LOGS, f'frame_{name}_{tag}.png')
        run(['-ss', f'{max(0, min(tt, dur - 0.2)):.2f}', '-i', chase, '-frames:v', '1', png])
        frames.append(png)
    sheet = os.path.join(LOGS, f'water_frames_{tag}.png')
    inputs = sum([['-i', p] for p in frames], [])
    fc = ''.join(f'[{i}:v]scale=640:-1,drawtext=text=\'{os.path.basename(p).split("_")[1]}\':x=10:y=10:fontsize=28:fontcolor=white:box=1:boxcolor=black@0.5[v{i}];'
                 for i, p in enumerate(frames))
    fc += '[v0][v1][v2][v3]hstack=inputs=4[top];[v4][v5][v6][v7]hstack=inputs=4[bot];[top][bot]vstack'
    try:
        run(inputs + ['-filter_complex', fc, sheet])
    except subprocess.CalledProcessError:
        fc = ''.join(f'[{i}:v]scale=640:-1[v{i}];' for i in range(len(frames)))
        fc += '[v0][v1][v2][v3]hstack=inputs=4[top];[v4][v5][v6][v7]hstack=inputs=4[bot];[top][bot]vstack'
        run(inputs + ['-filter_complex', fc, sheet])
    print('wrote', os.path.basename(hl), 'and', os.path.basename(sheet))
    return 0


if __name__ == '__main__':
    sys.exit(main())
