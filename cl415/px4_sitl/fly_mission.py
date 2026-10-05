"""fly the cl415 in px4 sitl: takeoff, a circuit, approach and landing, all in AUTO.MISSION

    ~/px4_venv/bin/python /mnt/c/Users/jamil/Desktop/jim/uni/CL415-VIP/FireFightingUAVF26-27/cl415/px4_sitl/fly_mission.py [--water]
    (sim running: px4_sitl/run_sim.sh). a ground station can stay connected on 18570, this uses px4's api link.

the runway (or the take-off run on the water) heads east from the spawn point. offsets are metres east / north
of home. --water flies the short water pattern: take-off, two turns, a long shallow approach and a landing back
on the water near the start.
"""
import argparse
import math
import sys
import time

from pymavlink import mavutil

PATTERNS = {
    'runway': dict(takeoff=(350, 0, 40), circuit=[(800, 0, 60), (800, 400, 60), (-300, 400, 60), (-450, 150, 45)],
                   approach=(-450, 0, 25), touchdown=(60, 0, 0)),
    'water': dict(takeoff=(300, 0, 40), circuit=[(700, 0, 50), (700, 350, 50), (-300, 350, 50), (-520, 120, 40)],
                  approach=(-520, 0, 25), touchdown=(150, 0, 0)),
}


def offset(lat, lon, east, north):
    r = 6378137.0
    return lat + math.degrees(north / r), lon + math.degrees(east / (r * math.cos(math.radians(lat))))


def wait_msg(m, kind, timeout=30, cond=None):
    t0 = time.time()
    while time.time() - t0 < timeout:
        msg = m.recv_match(type=kind, blocking=True, timeout=1)
        if msg is not None and (cond is None or cond(msg)):
            return msg
    return None


def upload(m, items):
    m.mav.mission_clear_all_send(m.target_system, m.target_component, mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
    wait_msg(m, 'MISSION_ACK', 5)
    m.mav.mission_count_send(m.target_system, m.target_component, len(items), mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
    sent = set()
    while True:
        msg = wait_msg(m, ['MISSION_REQUEST_INT', 'MISSION_REQUEST', 'MISSION_ACK'], 10)
        if msg is None:
            raise RuntimeError('mission upload timed out')
        if msg.get_type() == 'MISSION_ACK':
            if msg.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
                raise RuntimeError(f'mission rejected: {msg.type}')
            return
        cmd, lat, lon, alt, p = items[msg.seq]
        # do-commands go in the mission frame, px4 rejects them in a global frame
        frame = (mavutil.mavlink.MAV_FRAME_MISSION if cmd == mavutil.mavlink.MAV_CMD_DO_LAND_START
                 else mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT)
        m.mav.mission_item_int_send(m.target_system, m.target_component, msg.seq,
                                    frame, cmd, 1 if msg.seq == 0 else 0, 1,
                                    p[0], p[1], p[2], p[3], int(lat * 1e7), int(lon * 1e7), alt,
                                    mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
        sent.add(msg.seq)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--url', default='udpin:127.0.0.1:14540')
    ap.add_argument('--timeout', type=float, default=900.0, help='wall-clock limit [s]')
    ap.add_argument('--water', action='store_true', help='water pattern (take-off and landing on the water)')
    a = ap.parse_args()
    pat = PATTERNS['water' if a.water else 'runway']
    TAKEOFF, CIRCUIT, APPROACH, TOUCHDOWN = pat['takeoff'], pat['circuit'], pat['approach'], pat['touchdown']
    m = mavutil.mavlink_connection(a.url, source_system=245)
    m.wait_heartbeat(timeout=60)
    print(f'connected to system {m.target_system}')
    home = wait_msg(m, 'HOME_POSITION', 60)
    if home is None:
        m.mav.command_long_send(m.target_system, m.target_component, mavutil.mavlink.MAV_CMD_GET_HOME_POSITION,
                                0, 0, 0, 0, 0, 0, 0, 0)
        home = wait_msg(m, 'HOME_POSITION', 30)
    lat0, lon0 = home.latitude / 1e7, home.longitude / 1e7
    print(f'home {lat0:.6f} {lon0:.6f}')

    C = mavutil.mavlink
    items = []
    la, lo = offset(lat0, lon0, *TAKEOFF[:2])
    items.append((C.MAV_CMD_NAV_TAKEOFF, la, lo, TAKEOFF[2], (8, 0, 0, float('nan'))))
    for e, n, h in CIRCUIT:
        la, lo = offset(lat0, lon0, e, n)
        items.append((C.MAV_CMD_NAV_WAYPOINT, la, lo, h, (0, 0, 0, float('nan'))))
    items.append((C.MAV_CMD_DO_LAND_START, 0, 0, 0, (0, 0, 0, 0)))
    la, lo = offset(lat0, lon0, *APPROACH[:2])
    items.append((C.MAV_CMD_NAV_WAYPOINT, la, lo, APPROACH[2], (0, 0, 0, float('nan'))))
    la, lo = offset(lat0, lon0, *TOUCHDOWN[:2])
    items.append((C.MAV_CMD_NAV_LAND, la, lo, 0, (0, 0, 0, float('nan'))))
    upload(m, items)
    print(f'mission uploaded: {len(items)} items')

    # ready to arm?
    t0 = time.time()
    while time.time() - t0 < 60:
        st = wait_msg(m, 'SYS_STATUS', 5)
        if st and st.onboard_control_sensors_health & C.MAV_SYS_STATUS_PREARM_CHECK:
            break
    custom = (4 << 16) | (4 << 24)                          # px4 AUTO / MISSION
    m.mav.command_long_send(m.target_system, m.target_component, C.MAV_CMD_DO_SET_MODE, 0,
                            C.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4, 4, 0, 0, 0, 0)
    time.sleep(1)
    for _ in range(10):
        m.mav.command_long_send(m.target_system, m.target_component, C.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                                1, 0, 0, 0, 0, 0, 0)
        ack = wait_msg(m, 'COMMAND_ACK', 3, lambda x: x.command == C.MAV_CMD_COMPONENT_ARM_DISARM)
        if ack and ack.result == C.MAV_RESULT_ACCEPTED:
            break
        time.sleep(2)
    else:
        print('could not arm')
        return 2
    print('armed, mission running')

    t0 = time.time()
    last = 0
    armed_seen, landed = True, False
    state = {}
    while time.time() - t0 < a.timeout:
        msg = m.recv_match(blocking=True, timeout=1)
        if msg is None:
            continue
        k = msg.get_type()
        if k == 'STATUSTEXT':
            print(f'  [px4] {msg.text}')
        elif k in ('VFR_HUD', 'ATTITUDE', 'MISSION_CURRENT', 'GLOBAL_POSITION_INT', 'HEARTBEAT', 'EXTENDED_SYS_STATE'):
            if k == 'HEARTBEAT' and msg.get_srcSystem() != m.target_system:
                continue
            state[k] = msg
        if k == 'HEARTBEAT' and msg.get_srcSystem() == m.target_system:
            armed = bool(msg.base_mode & C.MAV_MODE_FLAG_SAFETY_ARMED)
            if armed_seen and not armed and time.time() - t0 > 20:
                landed = True
                break
        if time.time() - last > 3 and 'VFR_HUD' in state and 'ATTITUDE' in state and 'GLOBAL_POSITION_INT' in state:
            last = time.time()
            h, at, gp = state['VFR_HUD'], state['ATTITUDE'], state['GLOBAL_POSITION_INT']
            seq = state['MISSION_CURRENT'].seq if 'MISSION_CURRENT' in state else -1
            e = (gp.lon / 1e7 - lon0) * math.radians(1) * 6378137 * math.cos(math.radians(lat0))
            n = (gp.lat / 1e7 - lat0) * math.radians(1) * 6378137
            print(f'  t {time.time() - t0:5.0f}s wp {seq} pos e {e:6.0f} n {n:5.0f} alt {gp.relative_alt / 1000:5.1f} '
                  f'as {h.airspeed:4.1f} gs {h.groundspeed:4.1f} thr {h.throttle:3d}% '
                  f'roll {math.degrees(at.roll):5.1f} pitch {math.degrees(at.pitch):5.1f} climb {h.climb:4.1f}')
    print('landed and disarmed' if landed else 'timed out')
    return 0 if landed else 1


if __name__ == '__main__':
    sys.exit(main())
