#!/usr/bin/env python3
import math
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool

from serial_protocol import SerialLink


class SerialBridge(Node):
    def __init__(self):
        super().__init__('mooncar_serial_bridge')
        self.declare_parameter('port', '')
        self.declare_parameter('encoder_ticks_per_rev', 0)
        self.ticks_per_rev = self.get_parameter('encoder_ticks_per_rev').value
        if self.ticks_per_rev <= 0 or not self.get_parameter('port').value:
            raise ValueError('Set port and measured encoder_ticks_per_rev before hardware use')
        if self.get_parameter('use_sim_time').value:
            raise ValueError('Hardware serial bridge requires wall time')
        self.link = SerialLink(self.get_parameter('port').value)
        self.armed = False
        self.command = Twist()
        self.command_time = 0.0
        self.previous = None
        self.positions = [0.0]*4
        self.joints = self.create_publisher(JointState, '/joint_states', 10)
        self.status = self.create_publisher(DiagnosticArray, '/mooncar/hardware/status', 10)
        self.create_subscription(Twist, '/mooncar/cmd_vel_safe', self.on_command, 10)
        self.create_service(SetBool, '/mooncar/hardware/arm', self.on_arm)
        self.timer = self.create_timer(0.02, self.update, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def on_command(self, msg):
        self.command, self.command_time = msg, time.monotonic()

    def on_arm(self, request, response):
        self.armed = bool(request.data and self.link.healthy())
        response.success = not request.data or self.armed
        response.message = 'armed' if self.armed else 'disarmed; fresh fault-free telemetry required'
        return response

    def update(self):
        for telemetry in self.link.read():
            ticks = telemetry[:4]
            if self.previous is not None:
                for i in range(4):
                    delta = (ticks[i] - self.previous[i] + 2**31) % 2**32 - 2**31
                    self.positions[i] += delta * 2*math.pi/self.ticks_per_rev
            self.previous = ticks
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = ['front_left_wheel_joint', 'front_right_wheel_joint',
                        'rear_left_wheel_joint', 'rear_right_wheel_joint']
            msg.position = self.positions.copy()
            self.joints.publish(msg)
        if not self.link.healthy():
            self.armed = False
        fresh = time.monotonic() - self.command_time < 0.25
        self.link.send(self.command.linear.x if fresh else 0.0,
                       self.command.angular.z if fresh else 0.0, self.armed and fresh)
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = [DiagnosticStatus(
            name='serial_link', hardware_id=self.link.port.port,
            level=DiagnosticStatus.OK if self.link.healthy() else DiagnosticStatus.ERROR,
            message='armed' if self.armed else 'disarmed', values=[
                KeyValue(key='crc_errors', value=str(self.link.decoder.errors)),
                KeyValue(key='telemetry', value=str(self.link.telemetry)),
            ])]
        self.status.publish(msg)


def main():
    rclpy.init()
    node = None
    try:
        node = SerialBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                node.link.close()
            finally:
                node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
