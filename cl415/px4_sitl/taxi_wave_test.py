import argparse
import time
from pymavlink import mavutil


def wait_msg(m, kind, timeout=5, cond=None):
    t0 = time.time()
    while time.time() - t0 < timeout:
        msg = m.recv_match(type=kind, blocking=True, timeout=0.5)
        if msg is not None and (cond is None or cond(msg)):
            return msg
    return None


def send_manual(m, throttle):
    # MANUAL_CONTROL:
    # x = pitch, y = roll, z = throttle [0..1000], r = yaw
    z = int(max(0.0, min(1.0, throttle)) * 1000)

    m.mav.manual_control_send(
        m.target_system,
        0,      # pitch neutral
        0,      # roll neutral
        z,      # throttle
        0,      # yaw neutral
        0
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--url', default='udpin:127.0.0.1:14540')
    ap.add_argument('--throttle', type=float, default=0.20)
    ap.add_argument('--duration', type=float, default=8.0)
    ap.add_argument('--max-speed', type=float, default=3.0)
    a = ap.parse_args()

    m = mavutil.mavlink_connection(a.url, source_system=245)

    print("waiting for PX4...")
    m.wait_heartbeat(timeout=60)

    print(f"connected to system {m.target_system}")

    C = mavutil.mavlink

    # Wait for PX4 pre-arm checks while continuously
    # providing MANUAL_CONTROL and GCS heartbeat.
    print("waiting for pre-arm checks...")

    t0 = time.time()
    last_hb = 0.0
    prearm_ok = False

    while time.time() - t0 < 60:

        # MANUAL_CONTROL must be streamed continuously
        send_manual(m, 0.0)

        now = time.time()

        # Maintain a GCS heartbeat as well
        if now - last_hb >= 1.0:
            m.mav.heartbeat_send(
                C.MAV_TYPE_GCS,
                C.MAV_AUTOPILOT_INVALID,
                0,
                0,
                C.MAV_STATE_ACTIVE
            )
            last_hb = now

        st = m.recv_match(type='SYS_STATUS', blocking=False)

        if st and (
            st.onboard_control_sensors_health
            & C.MAV_SYS_STATUS_PREARM_CHECK
        ):
            prearm_ok = True
            break

        time.sleep(0.05)

    if not prearm_ok:
        print("ERROR: pre-arm checks did not become healthy")
        return 2

    print("pre-arm checks OK")

    # Send neutral manual input first
    for _ in range(10):
        send_manual(m, 0.0)
        time.sleep(0.1)

    # PX4 MANUAL mode
    print("switching to MANUAL mode")

    m.mav.command_long_send(
        m.target_system,
        m.target_component,
        C.MAV_CMD_DO_SET_MODE,
        0,
        C.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        1,      # PX4 main mode MANUAL
        0,
        0, 0, 0, 0
    )

    time.sleep(1)

    # Arm
    print("arming...")

    armed = False

    for _ in range(10):
        send_manual(m, 0.0)

        m.mav.command_long_send(
            m.target_system,
            m.target_component,
            C.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,
            0, 0, 0, 0, 0, 0
        )

        ack = wait_msg(
            m,
            'COMMAND_ACK',
            2,
            lambda x:
                x.command == C.MAV_CMD_COMPONENT_ARM_DISARM
        )

        if ack:
            print(f"arm ACK result = {ack.result}")

        # Print any PX4 status messages explaining the rejection
        while True:
            st = m.recv_match(type='STATUSTEXT', blocking=False)
            if st is None:
                break
            print("PX4:", st.text)

        if ack and ack.result == C.MAV_RESULT_ACCEPTED:
            armed = True
            break

        time.sleep(1)

    if not armed:
        print("ERROR: could not arm")
        return 2

    print("ARMED")
    print(
        f"Taxi test: throttle={a.throttle:.0%}, "
        f"duration={a.duration:.1f}s"
    )

    t0 = time.time()
    last_print = 0.0

    try:
        while time.time() - t0 < a.duration:

            send_manual(m, a.throttle)

            hud = m.recv_match(
                type='VFR_HUD',
                blocking=False
            )

            if hud is not None:

                elapsed = time.time() - t0

                if elapsed - last_print >= 0.5:
                    print(
                        f"t={elapsed:5.1f}s "
                        f"GS={hud.groundspeed:5.2f} m/s "
                        f"AS={hud.airspeed:5.2f} m/s "
                        f"thr={hud.throttle:3d}%"
                    )

                    last_print = elapsed

                # automatic safety cutoff
                if hud.groundspeed > a.max_speed:
                    print(
                        f"ABORT: groundspeed exceeded "
                        f"{a.max_speed:.1f} m/s"
                    )
                    break

            time.sleep(0.05)

    finally:

        print("throttle -> ZERO")

        for _ in range(20):
            send_manual(m, 0.0)
            time.sleep(0.05)

        print("disarming")

        m.mav.command_long_send(
            m.target_system,
            m.target_component,
            C.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0,
            0, 0, 0, 0, 0, 0
        )

        time.sleep(1)

        print("taxi test finished")

    return 0


if __name__ == '__main__':
    raise SystemExit(main())
