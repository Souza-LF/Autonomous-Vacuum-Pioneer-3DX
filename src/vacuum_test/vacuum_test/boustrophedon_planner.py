#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import Bool
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints


def quaternion_from_yaw(yaw):
    """Converte um ângulo Yaw (2D) para Quaternion (x, y, z, w)."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class BoustrophedonPlanner(Node):
    def __init__(self):
        super().__init__('boustrophedon_planner')

        # 0.38m garante que a linha verde fique FORA da zona de repulsão da parede no Nav2!
        self.wall_margin_m = 0.38    # 38 cm da parede (elimina o conflito de ficar olhando p/ os lados)
        self.stripe_step_m = 0.34    # 34 cm entre as faixas do ziguezague

        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )

        self.sub_map = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, map_qos)
        self.sub_done = self.create_subscription(
            Bool, '/mapping_done', self.done_callback, 10)
        self.sub_odom = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        self.pub_path = self.create_publisher(Path, '/boustrophedon_path', 10)
        self.waypoint_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')

        self.timer = self.create_timer(2.0, self.publish_cached_path)

        self.latest_map = None
        self.mapping_finished = False
        self.computed_path = None
        self.nav_started = False
        self.robot_x = 0.0
        self.robot_y = 0.0

        self.get_logger().info('Planejador Boustrophedon iniciado! Aguardando /map e /mapping_done...')

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def map_callback(self, msg):
        self.latest_map = msg
        if self.mapping_finished and self.computed_path is None:
            self.generate_boustrophedon_path()

    def done_callback(self, msg):
        if msg.data and not self.mapping_finished:
            self.mapping_finished = True
            self.get_logger().info('Sinal /mapping_done recebido! Gerando caminho Boustrophedon...')
            if self.latest_map is not None:
                self.generate_boustrophedon_path()

    def inflate_only_walls(self, grid, margin_cells):
        h, w = grid.shape
        walls = (grid >= 50)
        free_cells = (grid == 0)

        free_rows, free_cols = np.where(free_cells)
        if len(free_rows) == 0:
            return np.zeros((h, w), dtype=bool)

        r_min = int(np.percentile(free_rows, 2)) + margin_cells
        r_max = int(np.percentile(free_rows, 98)) - margin_cells
        c_min = int(np.percentile(free_cols, 2)) + margin_cells
        c_max = int(np.percentile(free_cols, 98)) - margin_cells

        room_box = np.zeros((h, w), dtype=bool)
        if r_max > r_min and c_max > c_min:
            room_box[r_min:r_max + 1, c_min:c_max + 1] = True

        inflated_walls = np.copy(walls)
        for dy in range(-margin_cells, margin_cells + 1):
            for dx in range(-margin_cells, margin_cells + 1):
                if dx * dx + dy * dy <= margin_cells * margin_cells:
                    inflated_walls |= np.roll(walls, shift=(dy, dx), axis=(0, 1))

        return room_box & (~inflated_walls)

    def grid_to_world(self, col, row, info):
        wx = info.origin.position.x + (col + 0.5) * info.resolution
        wy = info.origin.position.y + (row + 0.5) * info.resolution
        return wx, wy

    def create_pose_stamped(self, x, y, yaw, header):
        qx, qy, qz, qw = quaternion_from_yaw(yaw)
        pose = PoseStamped()
        pose.header = header
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def generate_boustrophedon_path(self):
        msg = self.latest_map
        info = msg.info
        res = info.resolution
        w, h = info.width, info.height

        grid = np.array(msg.data, dtype=np.int8).reshape((h, w))

        margin_cells = max(1, int(round(self.wall_margin_m / res)))
        step_cells = max(1, int(round(self.stripe_step_m / res)))

        safe_mask = self.inflate_only_walls(grid, margin_cells)

        row_counts = np.sum(safe_mask, axis=1)
        max_width = int(np.max(row_counts)) if len(row_counts) > 0 else 0
        if max_width < 5:
            self.get_logger().warn('Mapa sem largura suficiente!')
            return

        valid_rows = np.where(row_counts >= int(0.80 * max_width))[0]
        min_row, max_row = int(valid_rows[0]), int(valid_rows[-1])

        rows_to_visit = list(range(min_row, max_row + 1, step_cells))
        if (max_row - rows_to_visit[-1]) > (step_cells // 3):
            rows_to_visit.append(max_row)

        # Verifica se o robô está mais perto do topo (max_row) ou da base (min_row) da sala
        _, y_bottom = self.grid_to_world(0, rows_to_visit[0], info)
        _, y_top = self.grid_to_world(0, rows_to_visit[-1], info)
        if abs(self.robot_y - y_top) < abs(self.robot_y - y_bottom):
            rows_to_visit.reverse()

        # Verifica se o robô está mais perto da parede esquerda ou direita para iniciar da ponta certa
        first_row_cols = np.where(safe_mask[rows_to_visit[0], :])[0]
        x_left_0, _ = self.grid_to_world(int(first_row_cols[0]), rows_to_visit[0], info)
        x_right_0, _ = self.grid_to_world(int(first_row_cols[-1]), rows_to_visit[0], info)
        left_to_right = abs(self.robot_x - x_left_0) <= abs(self.robot_x - x_right_0)

        # Para o Nav2: apenas o ponto inicial e o ponto final de cada reta (com orientação suave na transição)
        nav_waypoints = []
        # Para o RViz2: pontos densos para visualizar a linha
        rviz_points = []

        for row in rows_to_visit:
            free_cols = np.where(safe_mask[row, :])[0]
            if len(free_cols) < 2:
                continue

            c_min, c_max = int(free_cols[0]), int(free_cols[-1])
            x_left, y_row = self.grid_to_world(c_min, row, info)
            x_right, _ = self.grid_to_world(c_max, row, info)

            x_start, x_end = (x_left, x_right) if left_to_right else (x_right, x_left)

            # Ponto de início e fim da faixa para o FollowWaypoints
            nav_waypoints.append((x_start, y_row))
            # Ponto intermediário no meio da sala para garantir que o Nav2 siga a reta
            nav_waypoints.append((0.5 * (x_start + x_end), y_row))
            nav_waypoints.append((x_end, y_row))

            # Pontos para o desenho no RViz2
            for t in np.linspace(0.0, 1.0, 10):
                rviz_points.append((x_start + t * (x_end - x_start), y_row))

            left_to_right = not left_to_right

        # Publica o Path no RViz2
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'

        for i in range(len(rviz_points)):
            x, y = rviz_points[i]
            nx, ny = rviz_points[i + 1] if i < len(rviz_points) - 1 else rviz_points[i]
            yaw = math.atan2(ny - y, nx - x) if i < len(rviz_points) - 1 else 0.0
            path_msg.poses.append(self.create_pose_stamped(x, y, yaw, path_msg.header))

        self.computed_path = path_msg
        self.pub_path.publish(self.computed_path)

        # Monta as poses enxutas para o FollowWaypoints do Nav2
        # Importante: no fim de cada reta, a pose já aponta na direção do próximo ponto!
        nav_poses = []
        for i in range(len(nav_waypoints)):
            x, y = nav_waypoints[i]
            if i < len(nav_waypoints) - 1:
                nx, ny = nav_waypoints[i + 1]
                yaw = math.atan2(ny - y, nx - x)
            else:
                px, py = nav_waypoints[i - 1]
                yaw = math.atan2(y - py, x - px)
            nav_poses.append(self.create_pose_stamped(x, y, yaw, path_msg.header))

        self.get_logger().info(
            f'Rota Boustrophedon pronta ({len(rows_to_visit)} faixas | {len(nav_poses)} waypoints)! '
            f'Iniciando pelo canto mais próximo do robô ({nav_waypoints[0][0]:.2f}, {nav_waypoints[0][1]:.2f}).'
        )

        self.send_to_nav2(nav_poses)

    def send_to_nav2(self, poses):
        if self.nav_started:
            return

        if not self.waypoint_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Servidor /follow_waypoints não encontrado!')
            return

        self.nav_started = True
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = poses

        self.get_logger().info(f'Enviando {len(poses)} waypoints para o Nav2!')
        send_goal_future = self.waypoint_client.send_goal_async(
            goal_msg, feedback_callback=self.nav_feedback_callback)
        send_goal_future.add_done_callback(self.nav_goal_response_callback)

    def nav_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Missão rejeitada pelo Nav2!')
            self.nav_started = False
            return

        self.get_logger().info('Missão ACEITA pelo Nav2! Iniciando limpeza...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.nav_result_callback)

    def nav_feedback_callback(self, feedback_msg):
        wp = feedback_msg.feedback.current_waypoint
        self.get_logger().info(f'Limpando... Waypoint atual: {wp}', throttle_duration_sec=4.0)

    def nav_result_callback(self, future):
        self.get_logger().info('LIMPEZA BOUSTROPHEDON CONCLUÍDA!')

    def publish_cached_path(self):
        if self.computed_path is not None:
            self.computed_path.header.stamp = self.get_clock().now().to_msg()
            self.pub_path.publish(self.computed_path)


def main(args=None):
    rclpy.init(args=args)
    node = BoustrophedonPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
