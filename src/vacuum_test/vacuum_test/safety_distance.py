#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class SafetyNode(Node):
    def __init__(self):
        super().__init__('safety_node')

        # Parâmetros
        self.declare_parameter('stop_distance', 0.3)        # m: abaixo disso bloqueia avanço
        self.declare_parameter('resume_distance', 0.6)      # m: histerese para liberar
        self.declare_parameter('front_half_angle_deg', 30.0)  # setor frontal analisado (±)
        self.declare_parameter('publish_rate', 20.0)        # Hz

        self.stop_distance = self.get_parameter('stop_distance').value
        self.resume_distance = self.get_parameter('resume_distance').value
        self.front_half_angle = math.radians(
            self.get_parameter('front_half_angle_deg').value)
        rate = self.get_parameter('publish_rate').value

        # Estado
        self.teleop_cmd = Twist()       # último comando do teclado
        self.front_min = float('inf')   # menor distância no setor frontal
        self.blocked = False

        # Pub/Sub
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Twist, '/teleop_cmd_vel', self.teleop_cb, 10)
        # qos_profile_sensor_data = BEST_EFFORT, compatível com a bridge do Gazebo
        self.create_subscription(LaserScan, '/scan', self.scan_cb,
                                 qos_profile_sensor_data)

        # Timer: publica SEMPRE, independente de chegar mensagem nova
        self.create_timer(1.0 / rate, self.control_loop)

        self.get_logger().info('Safety node iniciado.')

    def teleop_cb(self, msg: Twist):
        self.teleop_cmd = msg

    def scan_cb(self, scan: LaserScan):
        min_dist = float('inf')
        angle = scan.angle_min

        for r in scan.ranges:
            # normaliza o ângulo para [-pi, pi]
            a = math.atan2(math.sin(angle), math.cos(angle))
            if abs(a) <= self.front_half_angle:
                # ignora inf, nan e leituras fora dos limites do sensor
                if math.isfinite(r) and scan.range_min <= r <= scan.range_max:
                    min_dist = min(min_dist, r)
            angle += scan.angle_increment

        self.front_min = min_dist

    def control_loop(self):
        # Histerese para evitar ficar alternando bloqueado/liberado
        if not self.blocked and self.front_min < self.stop_distance:
            self.blocked = True
            self.get_logger().warn(
                f'Obstáculo a {self.front_min:.2f} m: bloqueando avanço!')
        elif self.blocked and self.front_min > self.resume_distance:
            self.blocked = False
            self.get_logger().info('Caminho livre: avanço liberado.')

        cmd = Twist()
        cmd.linear.x = self.teleop_cmd.linear.x
        cmd.linear.y = self.teleop_cmd.linear.y
        cmd.angular.z = self.teleop_cmd.angular.z

        # Bloqueia apenas avanço; ré e rotação continuam funcionando
        if self.blocked and cmd.linear.x > 0.0:
            cmd.linear.x = 0.0

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # para o robô ao encerrar
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
