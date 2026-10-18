#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


def euler_from_quaternion(q):
    """Converte quaternion da odometria para ângulo Yaw (radianos)."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle):
    """Mantém o ângulo no intervalo [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower_node')

        self.publisher_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.publisher_done = self.create_publisher(Bool, '/mapping_done', 10)

        self.subscription_scan = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.subscription_odom = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

	self.stop_count = 0

        # Loop de controle a 20 Hz (0.05s)
        self.timer = self.create_timer(0.05, self.control_loop)

        # Leituras dos setores do LiDAR
        self.front_dist = 10.0   # -30° a +30°
        self.right_dist = 10.0   # -90° a -60°
        self.r90_dist = 10.0     # Leitura exata a -90° (para checar paralelismo)
        self.r60_dist = 10.0     # Leitura exata a -60° (para checar paralelismo)

        # Pose atual via Odometria
        self.curr_x = 0.0
        self.curr_y = 0.0
        self.curr_yaw = 0.0
        self.odom_received = False

        # Referências para manobras (50 cm e giro de 90°)
        self.ref_x = 0.0
        self.ref_y = 0.0
        self.ref_yaw = 0.0

        # Variáveis do Critério de Parada (Volta Completa)
        self.start_x = None
        self.start_y = None
        self.left_start_zone = False
        self.departure_radius = 1.50
        self.arrival_radius = 0.45

        # Limites de distância (30 cm a 40 cm)
        self.min_dist = 0.30
        self.max_dist = 0.40
        self.target_dist = 0.35      # Alvo central: 35 cm
        self.wall_lost_dist = 0.75   # Acima de 75 cm na direita = parede realmente sumiu (quina aberta)
        self.lost_wall_counter = 0   # Filtro para confirmar que a parede sumiu de verdade

        # Estados da FSM:
        # 0: SEEK_WALL       -> Anda reto até detectar parede entre 30 e 40 cm na frente
        # 1: TURN_LEFT       -> Vira à esquerda até a frente liberar e a parede alinhar na direita
        # 2: FOLLOW_WALL     -> Segue a parede mantendo 30-40 cm na direita (-90° a -60°)
        # 3: CORNER_FORWARD  -> Quina aberta: anda 50 cm para frente
        # 4: CORNER_TURN_R   -> Quina aberta: vira 90° para a direita
        # 5: FINISHED        -> Mapeamento concluído
        self.state = 0
        self.get_logger().info('Iniciando no Estado 0: Buscando parede frontal (30-40 cm)...')

    def odom_callback(self, msg):
        self.curr_x = msg.pose.pose.position.x
        self.curr_y = msg.pose.pose.position.y
        self.curr_yaw = euler_from_quaternion(msg.pose.pose.orientation)
        self.odom_received = True

    def scan_callback(self, msg):
        num = len(msg.ranges)
        if num == 0:
            return

        def angle_to_idx(deg):
            rad = math.radians(deg)
            idx = int(round((rad - msg.angle_min) / msg.angle_increment))
            return max(0, min(num - 1, idx))

        # Direita: -90° a -60° | Frente: -30° a +30°
        r_start, r_end = angle_to_idx(-90.0), angle_to_idx(-60.0)
        f_start, f_end = angle_to_idx(-30.0), angle_to_idx(30.0)

        right_slice = msg.ranges[min(r_start, r_end) : max(r_start, r_end) + 1]
        front_slice = msg.ranges[min(f_start, f_end) : max(f_start, f_end) + 1]

        valid_right = [r for r in right_slice if not math.isinf(r) and not math.isnan(r) and r > 0.05]
        valid_front = [r for r in front_slice if not math.isinf(r) and not math.isnan(r) and r > 0.05]

        self.right_dist = min(valid_right) if valid_right else 10.0
        self.front_dist = min(valid_front) if valid_front else 10.0

        # Leituras nas pontas do setor direito (-90° e -60°) para saber se o robô está paralelo à parede
        # Se robô está paralelo, r60 ≈ r90 / cos(30°) = r90 * 1.155
        r90_slice = msg.ranges[min(r_start, r_end) : min(r_start, r_end) + 10]
        r60_slice = msg.ranges[max(r_start, r_end) - 10 : max(r_start, r_end) + 1]
        v90 = [r for r in r90_slice if not math.isinf(r) and not math.isnan(r) and r > 0.05]
        v60 = [r for r in r60_slice if not math.isinf(r) and not math.isnan(r) and r > 0.05]
        self.r90_dist = min(v90) if v90 else 10.0
        self.r60_dist = min(v60) if v60 else 10.0

    def save_reference_pose(self):
        self.ref_x = self.curr_x
        self.ref_y = self.curr_y
        self.ref_yaw = self.curr_yaw

    def get_distance_traveled(self):
        return math.hypot(self.curr_x - self.ref_x, self.curr_y - self.ref_y)

    def get_angle_turned(self):
        return abs(normalize_angle(self.curr_yaw - self.ref_yaw))

    def check_loop_closure(self):
        if self.start_x is None or self.start_y is None:
            return False

        dist_to_start = math.hypot(self.curr_x - self.start_x, self.curr_y - self.start_y)

        if not self.left_start_zone:
            if dist_to_start > self.departure_radius:
                self.left_start_zone = True
                self.get_logger().info(
                    f'Robô se afastou {dist_to_start:.2f}m do início. '
                    'Monitorando retorno ao ponto inicial...'
                )
            return False

        if dist_to_start <= self.arrival_radius:
            return True

        return False

    def control_loop(self):
        if not self.odom_received:
            return

        msg = Twist()

 	# =====================================================================
        # ESTADO 5: Percebe a volta, envia parada 3 vezes e libera o /cmd_vel
        # =====================================================================
        if self.state == 5:
            if self.stop_count < 3:
                # msg já nasce zerado (linear.x = 0.0, angular.z = 0.0)
                self.publisher_cmd_vel.publish(msg)

                done_msg = Bool()
                done_msg.data = True
                self.publisher_done.publish(done_msg)

                self.stop_count += 1
                self.get_logger().info(
                    f'Enviando parada e /mapping_done ({self.stop_count}/3)...'
                )

                if self.stop_count == 3:
                    self.get_logger().info(
                        'Robô parado! /cmd_vel liberado para o Nav2 assumir.'
                    )
                    self.timer.cancel()  # Desativa o loop para não competir com o Nav2
            return

        if self.state in (1, 2, 3, 4) and self.check_loop_closure():
            self.publisher_cmd_vel.publish(msg)
            self.get_logger().info(
                '========================================================\n'
                'VOLTA COMPLETA DETECTADA! Mapeamento das bordas concluído.\n'
                '========================================================'
            )
            self.state = 5
            return

        # =====================================================================
        # ESTADO 0: Anda para frente até F (-30° a +30°) detectar entre 30 e 40 cm
        # =====================================================================
        if self.state == 0:
            if self.front_dist > self.max_dist:
                msg.linear.x = 0.36
                msg.angular.z = 0.0
            else:
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                self.save_reference_pose()
                self.get_logger().info(
                    f'Parede frontal detectada a {self.front_dist:.2f}m! Virando à esquerda...'
                )
                self.state = 1

        # =====================================================================
        # ESTADO 1: Vira à esquerda até a frente liberar E a direita ler a parede
        # =====================================================================
        elif self.state == 1:
            msg.linear.x = 0.0
            msg.angular.z = 0.30  # Giro controlado à esquerda

            # Para estar paralelo à parede na direita:
            # 1) Frente livre (> 0.55m)
            # 2) Parede presente na direita (right_dist <= 0.48m)
            # 3) Feixe de -60° (r60) quase igual a r90 * 1.15 (robô paralelo à parede, não apontando p/ fora)
            is_parallel = abs(self.r60_dist - (self.r90_dist * 1.15)) < 0.12

            if self.front_dist > 0.55 and self.right_dist <= 0.48 and is_parallel:
                msg.angular.z = 0.0
                self.lost_wall_counter = 0

                if self.start_x is None:
                    self.start_x = self.curr_x
                    self.start_y = self.curr_y
                    self.get_logger().info(
                        f'Ponto inicial gravado em: X={self.start_x:.2f}m, Y={self.start_y:.2f}m'
                    )

                self.get_logger().info(
                    f'Alinhado com a parede! Direita={self.right_dist:.2f}m. Seguindo parede...'
                )
                self.state = 2

        # =====================================================================
        # ESTADO 2: Segue a parede mantendo 30-40 cm na direita (-90° a -60°)
        # =====================================================================
        elif self.state == 2:
            # PRIORIDADE 1: Parede à frente (<= 40 cm) -> Quina fechada, vira à esquerda
            if self.front_dist <= self.max_dist:
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                self.lost_wall_counter = 0
                self.save_reference_pose()
                self.get_logger().info(
                    f'Parede à frente ({self.front_dist:.2f}m)! Virando à esquerda...'
                )
                self.state = 1

            # PRIORIDADE 2: Checa se REALMENTE perdeu a parede da direita (Quina Aberta)
            # Precisa: (1) Frente livre (> 0.50m) E (2) Nenhuma parede na direita (< 0.75m)
            elif self.front_dist > 0.50 and self.right_dist > self.wall_lost_dist:
                self.lost_wall_counter += 1
                # Exige 4 leituras consecutivas (0.2s) sem parede na direita para confirmar quina aberta
                if self.lost_wall_counter >= 4:
                    self.lost_wall_counter = 0
                    self.save_reference_pose()
                    self.get_logger().info(
                        f'Quina aberta confirmada (Frente={self.front_dist:.2f}m, '
                        f'Direita={self.right_dist:.2f}m)! Avançando 50 cm...'
                    )
                    self.state = 3
                else:
                    msg.linear.x = 0.24
                    msg.angular.z = 0.0

            # PRIORIDADE 3: Parede está presente na direita (mesmo que tenha oscilado entre 25cm e 65cm)
            else:
                self.lost_wall_counter = 0
                msg.linear.x = 0.32

                # Controle PD geométrico:
                # 1) dist_error mantém a distância em 0.35m (entre 0.30m e 0.40m)
                dist_error = self.right_dist - self.target_dist
                # 2) angle_error mantém o robô paralelo à parede (evita que ele aponte p/ longe da parede)
                angle_error = self.r60_dist - (self.r90_dist * 1.15)

                turn_cmd = -(2 * dist_error + 1 * angle_error)
                msg.angular.z = max(-0.50, min(0.50, turn_cmd))

        # =====================================================================
        # ESTADO 3: Quina aberta (Parte 1) -> Anda 50 cm para frente
        # =====================================================================
        elif self.state == 3:
            if self.front_dist <= self.max_dist:
                msg.linear.x = 0.0
                self.save_reference_pose()
                self.get_logger().info('Parede frontal detectada durante os 50 cm! Virando à esquerda...')
                self.state = 1
            elif self.get_distance_traveled() < 0.50:
                msg.linear.x = 0.15
                msg.angular.z = 0.0
            else:
                msg.linear.x = 0.0
                self.save_reference_pose()
                self.get_logger().info('Avançou 50 cm! Virando 90° para a direita...')
                self.state = 4

        # =====================================================================
        # ESTADO 4: Quina aberta (Parte 2) -> Vira 90° para a direita
        # =====================================================================
        elif self.state == 4:
            msg.linear.x = 0.0
            msg.angular.z = -0.30  # Negativo = gira para a direita

            if self.get_angle_turned() >= 1.52:
                msg.angular.z = 0.0
                self.lost_wall_counter = 0
                if self.right_dist <= self.wall_lost_dist:
                    self.get_logger().info('Quina contornada e parede na direita! Retomando Wall Follow...')
                    self.state = 2
                else:
                    self.get_logger().info('Giro à direita concluído. Buscando parede à frente...')
                    self.state = 0

        self.publisher_cmd_vel.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop_msg = Twist()
        node.publisher_cmd_vel.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
