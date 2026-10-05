#!/usr/bin/env python3
"""wiggle every control surface and spin the props, to check the model in the gazebo gui

    ros2 launch cl415_description sim.launch.py          (in one terminal)
    python3 $(ros2 pkg prefix cl415_description)/share/cl415_description/scripts/surface_sweep.py

each surface sweeps +/- its limit for 4 s in turn (ailerons together, as a roll input), then both
motors ramp to 400 rad/s and back. ctrl-c stops it and zeroes everything.
"""
import math

import rclpy
from actuator_msgs.msg import Actuators
from rclpy.node import Node
from std_msgs.msg import Float64

LIM = {'aileron': math.radians(25), 'elevator': math.radians(25), 'rudder': math.radians(30)}


class Sweep(Node):
    def __init__(self):
        super().__init__('cl415_surface_sweep')
        self.pub = {k: self.create_publisher(Float64, f'/cl415/cmd/{k}', 10)
                    for k in ('left_aileron', 'right_aileron', 'elevator', 'rudder')}
        self.motor = self.create_publisher(Actuators, '/cl415/command/motor_speed', 10)
        self.t0 = self.get_clock().now()
        self.create_timer(0.02, self.tick)

    def send(self, la=0.0, ra=0.0, el=0.0, ru=0.0, w=0.0):
        for k, v in (('left_aileron', la), ('right_aileron', ra), ('elevator', el), ('rudder', ru)):
            self.pub[k].publish(Float64(data=v))
        m = Actuators()
        m.header.stamp = self.get_clock().now().to_msg()
        m.velocity = [w, w]
        self.motor.publish(m)

    def tick(self):
        t = (self.get_clock().now() - self.t0).nanoseconds * 1e-9
        phase, tt = int(t // 4) % 4, t % 4
        s = math.sin(2 * math.pi * tt / 4)
        if phase == 0:
            self.send(la=LIM['aileron'] * s, ra=-LIM['aileron'] * s)
        elif phase == 1:
            self.send(el=LIM['elevator'] * s)
        elif phase == 2:
            self.send(ru=LIM['rudder'] * s)
        else:
            self.send(w=400.0 * math.sin(math.pi * tt / 4))


def main():
    rclpy.init()
    n = Sweep()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.send()
    n.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
