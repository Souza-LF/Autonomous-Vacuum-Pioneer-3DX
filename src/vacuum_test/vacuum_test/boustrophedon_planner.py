#!/usr/bin/env python3
import math
import os
import numpy as np
from ament_index_python.packages import get_package_share_directory
from action_msgs.msg import GoalStatus
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import Bool
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses


BT_POR_DISTRO = {
    'humble': 'coverage_through_poses_humble.xml',
    'jazzy': 'coverage_through_poses_jazzy.xml',
}

STATUS_NOMES = {
    GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
    GoalStatus.STATUS_CANCELED: 'CANCELED',
    GoalStatus.STATUS_ABORTED: 'ABORTED',
}


def quaternion_from_yaw(yaw):
    """Converte um ângulo Yaw (2D) para Quaternion (x, y, z, w)."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class BoustrophedonPlanner(Node):
    def __init__(self):
        super().__init__('boustrophedon_planner')

        # Parâmetros de limpeza ajustáveis
        self.wall_margin_m = 0.42    # 42 cm de distância das paredes
        self.stripe_step_m = 0.40    # 40 cm entre as faixas do ziguezague
        self.point_spacing_m = 0.45  # Espaçamento entre waypoints na reta

        # QoS compatível com o /map do slam_toolbox
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )

        self.sub_map = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, map_qos)
        self.sub_done = self.create_subscription(
            Bool, '/mapping_done', self.done_callback, 10)

        self.pub_path = self.create_publisher(Path, '/boustrophedon_path', 10)

        # Action Client para enviar toda a trajetória ao Nav2 (NavigateThroughPoses)
        self.nav_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')

        self.timer = self.create_timer(2.0, self.publish_cached_path)

        self.latest_map = None
        self.mapping_finished = False
        self.computed_path = None
        self.nav_started = False

        self.get_logger().info(
            'Nó Boustrophedon + Nav2 iniciado! Aguardando /map e /mapping_done...'
        )

    def map_callback(self, msg):
        self.latest_map = msg
        if self.mapping_finished and self.computed_path is None:
            self.generate_boustrophedon_path()

    def done_callback(self, msg):
        if msg.data and not self.mapping_finished:
            self.mapping_finished = True
            self.get_logger().info('Sinal /mapping_done recebido! Calculando rota Boustrophedon...')
            if self.latest_map is not None:
                self.generate_boustrophedon_path()

    def inflate_obstacles(self, grid, margin_cells):
        h, w = grid.shape
        free_mask = (grid >= 0) & (grid < 50)
        safe_mask = np.copy(free_mask)

        for dy in range(-margin_cells, margin_cells + 1):
            for dx in range(-margin_cells, margin_cells + 1):
                if dx * dx + dy * dy <= margin_cells * margin_cells:
                    shifted = np.roll(free_mask, shift=(dy, dx), axis=(0, 1))
                    safe_mask &= shifted

        safe_mask[:margin_cells, :] = False
        safe_mask[h - margin_cells:, :] = False
        safe_mask[:, :margin_cells] = False
        safe_mask[:, w - margin_cells:] = False

        return safe_mask

    def grid_to_world(self, col, row, info):
        wx = info.origin.position.x + (col + 0.5) * info.resolution
        wy = info.origin.position.y + (row + 0.5) * info.resolution
        return wx, wy

    def generate_boustrophedon_path(self):
        msg = self.latest_map
        info = msg.info
        res = info.resolution
        w, h = info.width, info.height

        grid = np.array(msg.data, dtype=np.int8).reshape((h, w))

        margin_cells = max(1, int(round(self.wall_margin_m / res)))
        step_cells = max(1, int(round(self.stripe_step_m / res)))
        point_step_cells = max(1, int(round(self.point_spacing_m / res)))

        safe_mask = self.inflate_obstacles(grid, margin_cells)

        valid_rows = np.where(np.any(safe_mask, axis=1))[0]
        if len(valid_rows) == 0:
            self.get_logger().warn('Nenhuma célula segura encontrada!')
            return

        min_row, max_row = valid_rows[0], valid_rows[-1]

        waypoints_xy = []
        left_to_right = True

        for row in range(min_row, max_row + 1, step_cells):
            free_cols = np.where(safe_mask[row, :])[0]
            if len(free_cols) < 2:
                continue

            c_start, c_end = int(free_cols[0]), int(free_cols[-1])

            cols = list(range(c_start, c_end, point_step_cells))
            if cols[-1] != c_end:
                cols.append(c_end)

            if not left_to_right:
                cols.reverse()

            for c in cols:
                wx, wy = self.grid_to_world(c, row, info)
                waypoints_xy.append((wx, wy))

            left_to_right = not left_to_right

        if len(waypoints_xy) < 2:
            self.get_logger().warn('Poucos pontos gerados.')
            return

        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'

        for i in range(len(waypoints_xy)):
            x, y = waypoints_xy[i]
            if i < len(waypoints_xy) - 1:
                nx, ny = waypoints_xy[i + 1]
                yaw = math.atan2(ny - y, nx - x)
            else:
                px, py = waypoints_xy[i - 1]
                yaw = math.atan2(y - py, x - px)

            qx, qy, qz, qw = quaternion_from_yaw(yaw)

            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            path_msg.poses.append(pose)

        self.computed_path = path_msg
        self.pub_path.publish(self.computed_path)
        self.get_logger().info(
            f'Caminho Boustrophedon gerado ({len(path_msg.poses)} poses)! '
            'Enviando missão de cobertura para o Nav2...'
        )

        # Envia automaticamente os waypoints para o Nav2
        self.send_path_to_nav2(path_msg.poses)

    def send_path_to_nav2(self, poses):
        if self.nav_started:
            return

        self.get_logger().info('Aguardando servidor de ação /navigate_through_poses do Nav2...')
        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                'Nav2 não respondeu! Certifique-se de que o navigation_launch.py está rodando.'
            )
            return

        # BT própria: a padrão descarta waypoints a menos de 0,7 m (> espaçamento das faixas).
        # O formato do XML muda entre o Humble (BT.CPP v3) e o Jazzy (BT.CPP v4).
        distro = os.environ.get('ROS_DISTRO', '')
        bt_file = BT_POR_DISTRO.get(distro)
        if bt_file is None:
            self.get_logger().error(
                f'Não há BT de cobertura para ROS_DISTRO="{distro}" '
                f'(só {", ".join(BT_POR_DISTRO)}). Missão não enviada.'
            )
            return
        bt_path = os.path.join(
            get_package_share_directory('vacuum_test'), 'behavior_trees', bt_file)
        if not os.path.isfile(bt_path):
            self.get_logger().error(
                f'BT não encontrada em {bt_path}. Rode o colcon build de novo. Missão não enviada.'
            )
            return

        self.nav_started = True
        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = poses
        goal_msg.behavior_tree = bt_path
        self.get_logger().info(f'Usando a BT {bt_file} (ROS {distro}).')

        self.get_logger().info(f'Enviando {len(poses)} waypoints para o Nav2 iniciar a limpeza!')
        send_goal_future = self.nav_client.send_goal_async(
            goal_msg, feedback_callback=self.nav_feedback_callback)
        send_goal_future.add_done_callback(self.nav_goal_response_callback)

    def nav_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('A missão Boustrophedon foi rejeitada pelo Nav2!')
            self.nav_started = False
            return

        self.get_logger().info('Missão aceita pelo Nav2! Robô aspirando a sala...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.nav_result_callback)

    def nav_feedback_callback(self, feedback_msg):
        remaining = feedback_msg.feedback.number_of_poses_remaining
        # Loga apenas de tempos em tempos ou quando faltar pouco
        if remaining % 10 == 0:
            self.get_logger().info(f'Limpeza em andamento... Waypoints restantes: {remaining}')

    def nav_result_callback(self, future):
        result = future.result()
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            status = STATUS_NOMES.get(result.status, str(result.status))
            # O Humble não tem error_code no resultado (é std_msgs/Empty); o Jazzy tem
            error_code = getattr(result.result, 'error_code', None)
            detalhe = (
                f', error_code={error_code} {getattr(result.result, "error_msg", "")}'
                if error_code is not None else ''
            )
            self.get_logger().error(
                f'Nav2 NÃO concluiu a limpeza (status={status}{detalhe}). '
                'Veja o terminal do Nav2.'
            )
            return
        self.get_logger().info(
            '========================================================\n'
            'LIMPEZA BOUSTROPHEDON CONCLUÍDA COM SUCESSO PELO NAV2!\n'
            '========================================================'
        )

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
