import rclpy
import numpy as np
import csv
from cssrlib.gnss import ecef2enu, pos2ecef
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from gnss_ros_standardization.msg import GnssSolution

class WaypointFollowerNode(Node):
    def __init__(self):
        super().__init__('waypoint_follower_node')

        # --- CSV記録用の設定 ---
        self.log_filename = 'recorded_path.csv'
        with open(self.log_filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp', 'ecef_x', 'ecef_y', 'ecef_z'])

        # --- 基準点とウェイポイントの設定 ---
        # 画像のWP1を基準点として使用
        self.base_llh = np.array([35.69010481, 140.021192, 52.07312653])
        self.base_ecef = pos2ecef(self.base_llh, isdeg=True)
        
        # 画像から読み取った9つの座標 (LLH)
        waypoints_llh = np.array([
            [35.69010481, 140.021192, 52.07312653], # WP1
            [35.69006959, 140.02116,  52.01651243], # WP2
            [35.68997837, 140.0211486, 52.04321942], # WP3
            [35.689964,   140.0212289, 52.02832889], # WP4
            [35.68993376, 140.0213222, 52.04901289], # WP5
            [35.68994319, 140.0213661, 52.05799446], # WP6
            [35.68998468, 140.0214177, 51.96605622], # WP7
            [35.69005931, 140.0213192, 51.94683237], # WP8
            [35.69012819, 140.0212156, 51.96348143]  # WP9
        ])

        # LLHをENU（東・北・上）座標に変換
        self.waypoints_enu = []
        for wp in waypoints_llh:
            ecef = pos2ecef(wp, isdeg=True)
            enu = ecef2enu(self.base_llh, ecef - self.base_ecef)
            self.waypoints_enu.append(enu)
        self.waypoints_enu = np.array(self.waypoints_enu)

        # --- パラメータ ---
        self.Kp = -0.15
        self.Ki = 0.0
        self.target_speed = 0.6
        self.current_idx = 0
        self.integral_err = 0.0
        self.prev_time = self.get_clock().now()
        
        # --- サブスクライバ / パブリッシャ ---
        self.sub_pos = self.create_subscription(GnssSolution, '/gnss/solution', self.control_callback, 10)
        self.pub_drive = self.create_publisher(AckermannDriveStamped, '/ackermann_cmd', 10)

    def control_callback(self, msg):
        current_ecef = np.array([msg.pos_ecef.x, msg.pos_ecef.y, msg.pos_ecef.z])

        # CSV記録
        timestamp = self.get_clock().now().nanoseconds * 1e-9
        with open(self.log_filename, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([timestamp, current_ecef[0], current_ecef[1], current_ecef[2]])

        # 角度計算 (現在の方位)
        if hasattr(self, 'prev_ecef'):
            diff = current_ecef - self.prev_ecef
            enu_diff = ecef2enu(self.base_llh, diff)
            current_yaw = np.arctan2(enu_diff[0], enu_diff[1])
        else:
            current_yaw = 0.0
        self.prev_ecef = current_ecef
        
        # 目標点取得
        target_enu = self.waypoints_enu[self.current_idx]

        # 現在位置をENUへ変換
        enu_curr = ecef2enu(self.base_llh, current_ecef - self.base_ecef)
        dx = target_enu[0] - enu_curr[0]
        dy = target_enu[1] - enu_curr[1]
        
        # PID制御計算
        target_yaw = np.arctan2(dx, dy)
        angle_err = (target_yaw - current_yaw + np.pi) % (2 * np.pi) - np.pi
        
        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds * 1e-9
        self.prev_time = now
        
        self.integral_err = np.clip(self.integral_err + angle_err * dt, -0.5, 0.5)
        steering = (self.Kp * angle_err) + (self.Ki * self.integral_err)

        # パブリッシュ
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = float(np.clip(steering, -0.5, 0.5))
        drive_msg.drive.speed = self.target_speed
        self.pub_drive.publish(drive_msg)

        # 次のウェイポイントへ更新
        if np.sqrt(dx**2 + dy**2) < 0.5: # 閾値を0.5mに設定
            if self.current_idx < len(self.waypoints_enu) - 1:
                self.current_idx += 1

def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()