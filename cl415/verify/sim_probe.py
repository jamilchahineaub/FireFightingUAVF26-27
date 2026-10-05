#!/usr/bin/env python3
"""record the cl415 in gazebo (through the ros 2 bridge) and optionally send commands on a schedule

    python3 sim_probe.py <out.csv> <sim seconds> [actuate]

writes one row per odometry message: t, x, y, z, roll, pitch, yaw, vx, vy, vz, plus the latest
joint positions/velocities. with "actuate" it also drives the control surfaces and motors:
    t >= 3 s   left aileron +0.30, right aileron -0.30, elevator +0.20, rudder +0.40 rad
    t >= 6 s   elevator +1.00 rad (past its 0.436 limit, to see the limit hold)
    t >= 8 s   both motors 600 rad/s
    t >= 14 s  motors off
"""
import csv
import math
import sys

import rclpy
from actuator_msgs.msg import Actuators
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

JOINTS = ['left_aileron_joint', 'right_aileron_joint', 'elevator_joint', 'rudder_joint',
          'left_propeller_joint', 'right_propeller_joint']


def rpy(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


class Probe(Node):
    def __init__(self, path, duration, actuate):
        super().__init__('cl415_probe', parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.f = open(path, 'w', newline='')
        self.w = csv.writer(self.f)
        self.w.writerow(['t', 'x', 'y', 'z', 'roll', 'pitch', 'yaw', 'vx', 'vy', 'vz'] +
                        [f'{j}_pos' for j in JOINTS] + [f'{j}_vel' for j in JOINTS])
        self.duration = duration
        self.actuate = actuate
        self.t0 = None
        self.js = {}
        self.done = False
        self.create_subscription(Odometry, '/cl415/odometry', self.on_odom, 50)
        self.create_subscription(JointState, '/joint_states', self.on_js, 50)
        self.pub = {k: self.create_publisher(Float64, f'/cl415/cmd/{k}', 10)
                    for k in ('left_aileron', 'right_aileron', 'elevator', 'rudder')}
        self.motor = self.create_publisher(Actuators, '/cl415/command/motor_speed', 10)

    def on_js(self, m):
        for n, p, v in zip(m.name, m.position, m.velocity if m.velocity else [0.0] * len(m.name)):
            self.js[n] = (p, v)

    def command(self, t):
        surf = {'left_aileron': 0.0, 'right_aileron': 0.0, 'elevator': 0.0, 'rudder': 0.0}
        if t >= 3.0:
            surf = {'left_aileron': 0.30, 'right_aileron': -0.30, 'elevator': 0.20, 'rudder': 0.40}
        if t >= 6.0:
            surf['elevator'] = 1.0
        for k, v in surf.items():
            self.pub[k].publish(Float64(data=v))
        a = Actuators()
        a.header.stamp = self.get_clock().now().to_msg()
        w = 600.0 if 8.0 <= t < 14.0 else 0.0
        a.velocity = [w, w]
        self.motor.publish(a)

    def on_odom(self, m):
        t = m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec
        if self.t0 is None:
            self.t0 = t
        t -= self.t0
        if self.actuate:
            self.command(t)
        p = m.pose.pose.position
        r, pi, y = rpy(m.pose.pose.orientation)
        v = m.twist.twist.linear
        row = [f'{t:.4f}', p.x, p.y, p.z, r, pi, y, v.x, v.y, v.z]
        row += [self.js.get(j, (float('nan'), 0))[0] for j in JOINTS]
        row += [self.js.get(j, (0, float('nan')))[1] for j in JOINTS]
        self.w.writerow(row)
        if t >= self.duration:
            self.done = True


def main():
    path, dur = sys.argv[1], float(sys.argv[2])
    rclpy.init()
    n = Probe(path, dur, len(sys.argv) > 3 and sys.argv[3] == 'actuate')
    while rclpy.ok() and not n.done:
        rclpy.spin_once(n, timeout_sec=0.5)
    n.f.close()
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
