import numpy as np
from .rpy2rot import rpy2rot
from .rpy2rot_derivative import dRpy2rot_dphi, dRpy2rot_dtheta, dRpy2rot_dpsi
def skew(v):
    return np.array([[    0, -v[2],  v[1]],
                     [ v[2],     0, -v[0]],
                     [-v[1],  v[0],     0]])
class KalmanFilter:
    def __init__(self, drone_model, state, dt: float = 0.01):
        
        # EKF 상태 벡터, 공분산, 노이즈
        self.x_est = np.zeros((18,1))
        self.prev_x_est = np.zeros((18,1))  
        self.first_update = True
        self.P     = np.eye(18)*1e-4
        self.Q     = np.eye(18)*1e-6
        # 측정 노이즈
        sigma_acc  = 0.001
        sigma_euler_acc =0.001
        sigma_gyro = 0.001
        sigma_gps  = 0.003
        sigma_mag  = 0.003
        R_gps  = sigma_gps**2 * np.eye(3)
        R_euler_acc = sigma_euler_acc**2*np.eye(2)
        R_mag  = sigma_mag**2 * np.eye(1)
        R_gyro = sigma_gyro**2 * np.eye(3)
        R_acc = sigma_acc**2 *np.eye(3)
        R_vel = sigma_gps**2 * np.eye(3)  # GPS 속도 측정 노이즈
        
        # 전체 공분산 행렬 초기화
        R_imu = np.zeros((15, 15))

        # 매핑 (슬라이싱 기반)
        R_imu[0:3, 0:3] = R_gps
        R_imu[3:5, 3:5] = R_euler_acc
        R_imu[5, 5] = R_mag
        R_imu[6:9, 6:9] = R_vel
        R_imu[9:12, 9:12] = R_gyro
        R_imu[12:15, 12:15] = R_acc
        self.R_imu = R_imu
        # # 동역학 파라미터 (외부에서 설정 필요)  18-state estimation 에서는 feedback linearization 에 사용된 모델 이용
        # self.m_t = None      # 총 질량
        # self.F_ab = None     # 3×4 힘 매핑 행렬
        # self.Tau_ab = None   # 3×4 토크 매핑 행렬
        # self.I_tot = None    # 3×3 관성 모멘트 행렬
        # self.w_m = None      # 입력 모터 속도 벡터 (4×1)
        self.drone_model = drone_model
        self.state = state
        self.dt = 0.01 # self.drone_model.current_time-self.drone_model.prev_time
        self.angular_jerk = np.zeros((3,1))  # 이전 각속도 저장용
    


    def euler_acc(self, imu_msg, u: np.ndarray = None):  ## create roll,pitch reading from accelerometer
        acc = np.array([[imu_msg.linear_acceleration.x],
                        [imu_msg.linear_acceleration.y],
                        [imu_msg.linear_acceleration.z]])
        acc_by_grav = acc - (1/self.drone_model.m_t)*self.drone_model.F_ab @ self.state.w_d**2
        phi_acc = np.arctan2(acc_by_grav[1], acc_by_grav[2])
        theta_acc = np.arctan2(
            -acc_by_grav[0],
            np.sqrt(acc_by_grav[1]**2 + acc_by_grav[2]**2)
        )
        euler_acc = np.array([phi_acc, theta_acc])
        return euler_acc
        
    def predict(self, v: np.ndarray = None):
        
        A = np.block([
            [np.eye(6), self.dt*np.eye(6), 0.5*self.dt**2*np.eye(6)],
            [np.zeros((6,6)),np.zeros((6,6)),self.dt*np.eye(6)],
            [np.zeros((6,6)), np.zeros((6,6)), np.eye(6)]
        ])
        B = np.vstack([
            (1/6) * self.dt**3 * np.eye(6),
            (1/2) * self.dt**2 * np.eye(6),
            self.dt * np.eye(6)
        ])
        
        self.x_est = A.dot(self.x_est)  + B.dot(v) if v is not None else A.dot(self.x_est)
        
        self.P     = A.dot(self.P).dot(A.T) + self.Q
    def update(self, imu_msg, gps_msg, mag_msg, u: np.ndarray = None):
        phi = self.state.phi_hat
        theta = self.state.theta_hat
        psi = self.state.psi_hat
        R = rpy2rot(phi,theta,psi)
        R_T = R.T
        # 3x1 으로 확실히 reshape 보장
        acc = R_T @ (np.array([[-imu_msg.linear_acceleration.x],
                                [imu_msg.linear_acceleration.y],
                                [-imu_msg.linear_acceleration.z]]).reshape((3, 1)))  + np.array([0,0,9.81]).reshape(-1,1)

        gyro = np.array([[-imu_msg.angular_velocity.x],
                        [imu_msg.angular_velocity.y],
                        [-imu_msg.angular_velocity.z]]).reshape((3, 1))

        gps = np.array([[gps_msg.pose.pose.position.x],
                        [-gps_msg.pose.pose.position.y],
                        [-gps_msg.pose.pose.position.z]]).reshape((3, 1))

        mag = np.array([[mag_msg.magnetic_field.x],
                        [mag_msg.magnetic_field.y],                        # added x,y for test
                        [mag_msg.magnetic_field.z]]).reshape((3, 1))
        vel = np.array([[gps_msg.twist.twist.linear.x],
                        [-gps_msg.twist.twist.linear.y],
                        [-gps_msg.twist.twist.linear.z]]).reshape((3,1))
        # euler_acc = self.euler_acc(imu_msg, u).reshape((2, 1)) trial using true euler

        # 최종 관측 벡터
        z_k = np.vstack((gps, mag,vel, gyro, acc))

        H = np.block([
            [np.eye(3), np.zeros((3,15))],
            [np.zeros((3,3)), np.eye(3), np.zeros((3,12))],
            [np.zeros((3,6)), np.eye(3), np.zeros((3,9))],
            [np.zeros((3,9)), np.eye(3), np.zeros((3,6))],
            [np.zeros((3,12)), np.eye(3), np.zeros((3,3))]
        ])
        S = H.dot(self.P).dot(H.T) + self.R_imu
        K = self.P.dot(H.T).dot(np.linalg.inv(S))
        y = z_k - H.dot(self.x_est)
        self.x_est = self.x_est + K.dot(y)
        self.x_est[0] = gps_msg.pose.pose.position.x
        self.x_est[1] = -gps_msg.pose.pose.position.y
        self.x_est[2] = -gps_msg.pose.pose.position.z
        self.x_est[3] = -mag_msg.magnetic_field.x
        self.x_est[4] = -mag_msg.magnetic_field.y
        self.x_est[5] = mag_msg.magnetic_field.z
        self.x_est[6] = gps_msg.twist.twist.linear.x
        self.x_est[7] = -gps_msg.twist.twist.linear.y
        self.x_est[8] = -gps_msg.twist.twist.linear.z
        self.x_est[9] = imu_msg.angular_velocity.x
        self.x_est[10] = -imu_msg.angular_velocity.y
        self.x_est[11] = -imu_msg.angular_velocity.z
        if self.first_update:
            self.x_est[12:18] = 0  # 초기 가속도 및 각가속도 추정값
            self.first_update = False
            self.angular_jerk[0] = 0
            self.angular_jerk[1] = 0
            self.angular_jerk[2] = 0
        else:
            

            self.x_est[12] = (self.x_est[6]-self.prev_x_est[6])/self.dt  # x_ddot
            self.x_est[13] = (self.x_est[7]-self.prev_x_est[7])/self.dt
            self.x_est[14] = (self.x_est[8]-self.prev_x_est[8])/self.dt
            self.x_est[15] = (self.x_est[9]-self.prev_x_est[9])/self.dt  # x_ddot
            self.x_est[16] = (self.x_est[10]-self.prev_x_est[10])/self.dt
            self.x_est[17] = (self.x_est[11]-self.prev_x_est[11])/self.dt

            if(self.x_est[15] == 0 and self.x_est[16] == 0 and self.x_est[17] == 0):
                self.x_est[15:18] = self.angular_jerk[0:3]  # 각속도 jerk가 0인 경우 이전 값을 유지

            else:
                self.angular_jerk[0:3] = self.x_est[15:18].copy()

        self.prev_x_est = self.x_est.copy()  # 이전 상태 저장
        
        self.P     = (np.eye(18)-K.dot(H)).dot(self.P)
        
     