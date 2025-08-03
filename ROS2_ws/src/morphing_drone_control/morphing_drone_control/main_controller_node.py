import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from sensor_msgs.msg import MagneticField
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
import transforms3d
import numpy as np


from morphing_drone_control.guidance_manager import GuidanceManager
from morphing_drone_control.mode_controller import ModeController
from morphing_drone_control.motor_controller import MotorController
from morphing_drone_control.kalman_filter import KalmanFilter
from morphing_drone_control.fault_detection import FaultDetection
from morphing_drone_control.drone_model import DroneModel

class DroneState:
    def __init__(self):
        self.x_hat = 0.0
        self.y_hat = 0.0
        self.z_hat = 0.0

        self.x_dot_hat = 0.0
        self.y_dot_hat = 0.0
        self.z_dot_hat = 0.0
        
        self.x_ddot_hat = 0.0
        self.y_ddot_hat = 0.0
        self.z_ddot_hat = 0.0

        self.phi_hat   = 0.0
        self.theta_hat = 0.0
        self.psi_hat   = 0.0

        self.phi_dot_hat   = 0.0
        self.theta_dot_hat = 0.0
        self.psi_dot_hat   = 0.0
        
        self.phi_ddot_hat   = 0.0
        self.theta_ddot_hat = 0.0
        self.psi_ddot_hat   = 0.0


        self.v =np.array([
            0,0,0,0,0,0
        ]).reshape(-1,1)

        self.alpha = np.zeros((4, 1))
        self.alpha_dot = np.zeros((4, 1))
        self.beta  = np.zeros((4, 1))
        self.beta_dot  = np.zeros((4, 1))
        self.w_d = np.array([
            -1400.0, 1400.0, -1400.0, 1400.0
        ]).reshape(-1,1)

        self.mode = "X"
        
        # 센서


class MorphingDroneController(Node):
    def __init__(self):
        super().__init__('morphing_drone_controller')  # ROS 2 노드 초기화
        
        # 1) 파라미터 선언 -- 수정필요
        param_defaults = {
            'bodyMass':      0.733,
            'armMass':       0.139,
            'armcmLength':   0.139113738822933,
            'armLength':     0.1595,
            'bodyLength':    0.114552,
            'Ixxa':          2.190839539142159e-04,
            'Ixya':          -4.301600091384631e-05,
            'Ixza':          6.131159813951677e-05,
            'Iyya':          3.986596961705956e-04,
            'Iyza':          6.459346045048527e-05,
            'Izza':          3.329336946986987e-04,
            'Ixxb':          0.001313892,
            'Iyyb':          0.001655877,
            'Izzb':          0.002162122,
            'Ixzb':          -1.835200000000000e-05,
            'ThrustCoeff':   2.496609523809524e-06,
            'DragCoeff':     2.755446712018140e-07    # needs proper parameters
        }
        for name, default in param_defaults.items():
            self.declare_parameter(name, default)
        params = {name: self.get_parameter(name).value for name in param_defaults}

        # 2) 상태, 필터 등 class 
        self.state = DroneState()
        self.drone_model = DroneModel(params, self.state)
        self.drone_model.update(alpha=self.state.alpha)
        self.kf = KalmanFilter(self.drone_model, self.state)
        
        self.guidance = GuidanceManager(self.state)
        self.fault_detection = FaultDetection(self, self.state)
        self.mode_controller = ModeController(self.state,self.guidance,self.drone_model,self.fault_detection)
        
        self.motor_controller = MotorController(self, self.state, self.drone_model)

        # 3) 센서 데이터 저장 변수
        self.imu_data = None
        self.gps_data = None
        self.mag_data = None

        # Subscribe to sensors
        self.create_subscription(Imu, '/imu_plugin/out', self.imu_callback, 10)
        self.create_subscription(Odometry, '/my_drone/odom', self.gps_callback, 10)
        # Magnetometer is synthesized from IMU yaw
        # self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.x_est_pub = self.create_publisher(Float32MultiArray, '/kalman/x_est', 10)
        self.force_pub  = self.create_publisher(Float32MultiArray, '/my_drone/force_cmd', 10)
        self.torque_pub = self.create_publisher(Float32MultiArray, '/my_drone/torque_cmd', 10)
        # Control loop timer (10ms)
        self.timer = self.create_timer(0.01, self.control_loop)
        

    def imu_callback(self, msg: Imu):
        # Store IMU data
        self.imu_data = msg
        # Extract yaw from quaternion
        q = msg.orientation
        roll, pitch, yaw = transforms3d.euler.quat2euler([q.w, q.x, q.y, q.z],axes='rxyz') # axes='rxyz' 안에 넣기
        # Synthesize magnetometer message with yaw
        mag = MagneticField()
        mag.header = msg.header
        mag.magnetic_field.x = -float(roll)
        mag.magnetic_field.y = float(pitch)
        mag.magnetic_field.z = -float(yaw)
        self.mag_data = mag

    def gps_callback(self, msg: Odometry):
        self.gps_data = msg

    # def joint_state_callback(self, msg: JointState):
    #     tilt_joint_names = ['tilt1_joint', 'tilt2_joint', 'tilt3_joint', 'tilt4_joint']
    #     beta_vals = np.zeros((4, 1))

    #     for i, name in enumerate(msg.name):
    #         if name in tilt_joint_names:
    #             index = tilt_joint_names.index(name)
    #             beta_vals[index][0] = msg.position[i]

    #     self.state.beta = beta_vals

    def publish_x_est(self):
        msg = Float32MultiArray()
        msg.data = self.x_est.flatten().tolist()  # 18개 원소
        self.ros_node.x_est_pub.publish(msg)

    def control_loop(self):
        # 모든 센서 데이터가 준비되었는지 확인
        if self.imu_data is None or self.gps_data is None or self.mag_data is None:
            return
        
        # 1) 동역학 파라미터 업데이트 -- 실제 식에 맞게 수정 필요
        # DroneState 클래스에서 w_d를 numpy (4,1) 형태로 저장해야 함 #여기 순서좀
        self.drone_model.update(
            alpha=self.state.alpha
        )
        # 2) Kalman Filter 추청 및 현재 상태에 반영

        # 예측, 갱신
        self.kf.predict(self.state.v)
        self.kf.update(self.imu_data, self.gps_data, self.mag_data, self.state.w_d)
        # 칼만 추정값 퍼블리쉬
        msg = Float32MultiArray()
        msg.data = self.kf.x_est.flatten().tolist()
        self.x_est_pub.publish(msg)
        
        
        # state에 반영
        est = self.kf.x_est  # 18×1 추정 상태 벡터
        
        self.state.x_hat = est[0][0]
        self.state.y_hat = est[1][0]
        self.state.z_hat = est[2][0]
        self.state.x_dot_hat = est[6][0]
        self.state.y_dot_hat = est[7][0]
        self.state.z_dot_hat = est[8][0]
        self.state.x_ddot_hat = est[12][0]
        self.state.y_ddot_hat = est[13][0]
        self.state.z_ddot_hat = est[14][0]
        self.state.phi_hat = est[3][0]
        self.state.theta_hat = est[4][0]
        self.state.psi_hat = est[5][0]
        self.state.phi_dot_hat = est[9][0]
        self.state.theta_dot_hat = est[10][0]
        self.state.psi_dot_hat = est[11][0]
        self.state.phi_ddot_hat = est[15][0]
        self.state.theta_ddot_hat = est[16][0]
        self.state.psi_ddot_hat = est[17][0]
        # 2) Kalman Filter 추청 및 현재 상태에 반영 이거 루프 마지막으로 보내야 F_ab,Tau_ab 계산 됨
        self.kf.m_t    = self.drone_model.m_t
        self.kf.F_ab   = self.drone_model.F_ab
        self.kf.Tau_ab = self.drone_model.Tau_ab
        self.kf.I_tot  = self.drone_model.I_total
        self.kf.w_m    = self.state.w_d
        
        
        
        # TODO: 3) Navigation - Fault Detection
        # TODO: 4) Navigation - Mode Classification 
        # TODO: 5) Guidance(Ros2 와 Controller 좌표축 감안할것)

        # 6) Controller - 제어기에서 제어 출력 계산(w_d², α̇_d, β̇_d) 및 state에 업데이트
        self.mode_controller.update_abw()
        F_vec   = self.mode_controller.F
        Tau_vec = self.mode_controller.Tau

        f_msg = Float32MultiArray()
        # data 순서는 [Fx, Fy, Fz]
        f_msg.data = [float(F_vec[0,0]), float(F_vec[1,0]), float(F_vec[2,0])]
        self.force_pub.publish(f_msg)

        t_msg = Float32MultiArray()
        # data 순서는 [Tx, Ty, Tz]
        t_msg.data = [float(Tau_vec[0,0]), float(Tau_vec[1,0]), float(Tau_vec[2,0])]
        self.torque_pub.publish(t_msg)
        # sub - 충돌 체크 

        # 7) 모터 명령어 생성 및 PWM 신호 전송
        self.motor_controller.send_commands()
        
        
        
        

def main(args=None):
    rclpy.init(args=args)               # ROS 2 초기화
    node = MorphingDroneController()    # 노드 객체 생성
    rclpy.spin(node)                    # 노드 실행
    node.destroy_node()
    rclpy.shutdown()                    # 종료 시 ROS 종료

if __name__ == '__main__':
    main()