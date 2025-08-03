import numpy as np
from morphing_drone_control.rpy2rot import rpy2rot
from morphing_drone_control.rpy2rot_derivative import RPY2Rot_derivative


class ModeController:
    def __init__(self, state, guidance, drone_model,fault_detection):
        self.state = state
        self.guidance = guidance
        self.drone_model = drone_model
        self.fault_detection = fault_detection
        self.F   = np.zeros((3, 1), dtype=float)
        self.Tau = np.zeros((3, 1), dtype=float)
    
    def update_abw(self):
    # 변수정의

    # 0) 토픽으로 들어온 failnum 읽어서 mode 고정
        if self.fault_detection.failnum > 0:
            self.state.mode = 'Y'
        else:
            self.state.mode = 'X'
    # State 관련
        x = self.state.x_hat
        y = self.state.y_hat
        z = self.state.z_hat
        
        x_dot = self.state.x_dot_hat
        y_dot = self.state.y_dot_hat
        z_dot = self.state.z_dot_hat
        
        x_ddot = self.state.x_ddot_hat
        y_ddot = self.state.y_ddot_hat
        z_ddot = self.state.z_ddot_hat
        
        phi = self.state.phi_hat
        theta = self.state.theta_hat
        psi = self.state.psi_hat
        
        phi_dot = self.state.phi_dot_hat
        theta_dot = self.state.theta_dot_hat
        psi_dot = self.state.psi_dot_hat
        
        phi_ddot = self.state.phi_ddot_hat
        theta_ddot = self.state.theta_ddot_hat
        psi_ddot = self.state.psi_ddot_hat
        
        # 팔 각도 관련
        alpha = self.state.alpha
        # alpha[[0,2]]=-1*alpha[[2,0]]
        # alpha[[1,3]]=-1*alpha[[3,1]]
        beta = self.state.beta
        # beta[[0,2]]=beta[[2,0]]
        # beta[[1,3]]=beta[[3,1]]
        
        #여기는 아직 없음 따로 추가해줘야 함
        self.drone_model.update(alpha)
        I_prev = self.drone_model.prev_I_total
        I_cur = self.drone_model.cur_I_total
        dt = 0.01 #self.drone_model.current_time-self.drone_model.prev_time
        
        kf = self.drone_model.kf
        km = self.drone_model.km 
        al = self.drone_model.al
        bl = self.drone_model.bl
        m_t = self.drone_model.m_t
        
        w_m = self.state.w_d**2
        # w_m[[0,2]] = w_m[[2,0]]
        # w_m[[1,3]] = w_m[[3,1]]
        # Guidance 관련 (update 되면 해야 함)
        x_d = self.guidance.x_d 
        y_d = self.guidance.y_d
        z_d = self.guidance.z_d
        phi_d = self.guidance.phi_d
        theta_d = self.guidance.theta_d
        psi_d = self.guidance.psi_d
        
        if self.state.mode == "X":
            
            #Z-domain에서 v 값 계산
            z_current = np.array([
                [x,x_dot,x_ddot,y,y_dot,y_ddot,z,z_dot,z_ddot,phi,phi_dot,phi_ddot,theta,theta_dot,theta_ddot,psi, psi_dot, psi_ddot]
            ]).T
            z_ref = np.array([
                [x_d,0,0,y_d,0,0,z_d,0,0,phi_d,0,0,theta_d,0,0,psi_d,0,0]
            ]).T
            error = z_ref-z_current
  
            
            K_lqr = np.array([
                [ 3.1623,  4.4596,  2.9865,  0.0000, -0.0000, -0.0000, -0.0000, -0.0000,  0.0000, -0.0000, -0.0000, -0.0000, -0.0000, -0.0000, -0.0000,  0.0000,  0.0000, -0.0000],
                [ 0.0000,  0.0000,  0.0000,  3.1623,  4.4596,  2.9865, -0.0000, -0.0000, -0.0000,  0.0000,  0.0000,  0.0000, -0.0000, -0.0000, -0.0000,  0.0000,  0.0000,  0.0000],
                [-0.0000,  0.0000,  0.0000, -0.0000,  0.0000,  0.0000,  3.1623,  4.4596,  2.9865,  0.0000,  0.0000,  0.0000, -0.0000, -0.0000, -0.0000,  0.0000,  0.0000,  0.0000],
                [ 0.0000, -0.0000, -0.0000,  0.0000,  0.0000, -0.0000, -0.0000, -0.0000, -0.0000,  3.1623,  4.4596,  2.9865,  0.0000,  0.0000,  0.0000, -0.0000, -0.0000, -0.0000],
                [-0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000, -0.0000,  0.0000,  0.0000, -0.0000, -0.0000, -0.0000,  3.1623,  4.4596,  2.9865, -0.0000, -0.0000,  0.0000],
                [ 0.0000,  0.0000,  0.0000, -0.0000, -0.0000, -0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  3.1623,  4.4596,  2.9865]
            ])
            # K_lqr = np.array([      
            
            v_lqr = K_lqr@error
            self.state.v = v_lqr
            
            
            #I_d 값 계산(Numerical method)
            
            I_inv_cur = np.linalg.inv(I_cur)
            I_inv_prev = np.linalg.inv(I_prev)
            I_d = (I_inv_cur-I_inv_prev)/dt
            
            #F_a_b 값 계산
            p2x = np.array([
                [np.cos(-np.pi/4), -np.sin(-np.pi/4), 0],
                [np.sin(-np.pi/4), np.cos(-np.pi/4), 0],
                [0, 0, 1]
            ])
            F_a_b = np.array([
                [0, kf*np.sin(beta[0][0]), -kf*np.cos(beta[0][0])],
                [-kf*np.cos(alpha[1][0])*np.sin(beta[1][0]), -kf*np.sin(alpha[1][0])*np.sin(beta[1][0]), -kf*np.cos(beta[1][0])],
                [0, -kf*np.sin(beta[2][0]), -kf*np.cos(beta[2][0])],
                [kf*np.cos(alpha[3][0])*np.sin(beta[3][0]), kf*np.sin(alpha[3][0])*np.sin(beta[3][0]), -kf*np.cos(beta[3][0])]
            ]).T
            F_a_b = p2x@F_a_b
            
            #Tau_a_b 값 계산 #bl에 1/sqrt(2) 안곱하는 이유 궁금
            Tau_a_b = np.array([
                [0, kf * (bl+al) * np.cos(beta[0][0]) + km*np.sin(beta[0][0]), kf * (bl+al) * np.sin(beta[0][0]) - km * np.cos(beta[0][0])],
                [-kf* (bl+al*np.cos(alpha[1][0]))*np.cos(beta[1][0]) + km*np.cos(alpha[1][0])*np.sin(beta[1][0]), -kf*al*np.sin(alpha[1][0])*np.cos(beta[1][0])+km*np.sin(alpha[1][0])*np.sin(beta[1][0]),kf*al*(np.sin(alpha[1][0]))**2*np.sin(beta[1][0])+kf*(bl+al*np.cos(alpha[1][0]))*np.cos(alpha[1][0])*np.sin(beta[1][0])+ km*np.cos(beta[1][0])],
                [0,  -kf*(bl+al)*np.cos(beta[2][0]) - km*np.sin(beta[2][0]),kf*(bl+al)*np.sin(beta[2][0])-km*np.cos(beta[2][0])],
                [kf*(bl+al*np.cos(alpha[3][0]))*np.cos(beta[3][0])-km*np.cos(alpha[3][0])*np.sin(beta[3][0]), kf*al*np.sin(alpha[3][0])*np.cos(beta[3][0])-km*np.sin(alpha[3][0])*np.sin(beta[3][0]),kf*al*(np.sin(alpha[3][0]))**2*np.sin(beta[3][0])+kf*(bl+al*np.cos(alpha[3][0]))*np.cos(alpha[3][0])*np.sin(beta[3][0])+km*np.cos(beta[3][0])]
            ]).T
            Tau_a_b = p2x @ Tau_a_b
            #JR matrix 
            R = rpy2rot(phi,theta,psi)         # 3×3 회전 행렬
            R_T = R.T                      # RPY2Rot(obj.euler)'에 해당

            top_left = (1 / m_t) * R_T
            top_right = np.zeros((3, 3))
            bottom_left = np.zeros((3, 3))
            bottom_right = np.linalg.inv(I_cur)
            
            JR = np.block([
                [top_left,     top_right],
                [bottom_left,  bottom_right]
            ])
            
            #JR_dot matrix 
            dR = RPY2Rot_derivative(phi,theta,psi,phi_dot,theta_dot,psi_dot)
            top_left = (1/m_t)*dR
            top_right = np.zeros((3,3))
            bottom_left = np.zeros((3,3))
            #inv인지 zeros인지 확인
            bottom_right = np.zeros((3,3))
            
            JRdot = np.block([
                [top_left, top_right],
                [bottom_left,bottom_right]
            ])
            
            #J_beta, J_betadot 계산후 B matrix 
            J_beta = np.vstack((F_a_b,Tau_a_b)) # 6*4 matrix
            
            top = p2x@np.array([
                [0,  -kf*np.cos(alpha[1][0])*np.cos(beta[1][0])*w_m[1][0], 0, kf*np.cos(alpha[3][0])*np.cos(beta[3][0])*w_m[3][0]],
                [kf*np.cos(beta[0][0])*w_m[0][0],  -kf*np.sin(alpha[1][0])*np.cos(beta[1][0])*w_m[1][0], -kf*np.cos(beta[2][0])*w_m[2][0], kf*np.sin(alpha[3][0])*np.cos(beta[3][0])*w_m[3][0]],
                [kf*np.sin(beta[0][0])*w_m[0][0],kf*np.sin(beta[1][0])*w_m[1][0], kf*np.sin(beta[2][0])*w_m[2][0],kf*np.sin(beta[3][0])*w_m[3][0]]
            ])
            bottom = p2x@np.array([
                [0,kf*(bl+al*np.cos(alpha[1][0]))*np.sin(beta[1][0])*w_m[1][0] + km*np.cos(alpha[1][0])*np.cos(beta[1][0])*w_m[1][0],0,-kf*(bl+al*np.cos(alpha[3][0]))*np.sin(beta[3][0])*w_m[3][0] - km*np.cos(alpha[3][0])*np.cos(beta[3][0])*w_m[3][0]],
                [(-kf*(bl+al)*np.sin(beta[0][0]) + km*np.cos(beta[0][0]))*w_m[0][0], (kf*al*np.sin(alpha[1][0])*np.sin(beta[1][0])*w_m[1][0] + km*np.sin(alpha[1][0])*np.cos(beta[1][0]))*w_m[1][0],(kf*(bl+al)*np.sin(beta[2][0]) - km*np.cos(beta[2][0]))*w_m[2][0], -(kf*al*np.sin(alpha[3][0])*np.sin(beta[3][0]) + km*np.sin(alpha[3][0])*np.cos(beta[3][0]))*w_m[3][0]],
                [(kf*(bl+al)*np.cos(beta[0][0]) + km*np.sin(beta[0][0]))*w_m[0][0], (kf*al*(np.sin(alpha[1][0]))**2*np.cos(beta[1][0]) + kf*(bl+al*np.cos(alpha[1][0]))*np.cos(alpha[1][0])*np.cos(beta[1][0]) - km*np.sin(beta[1][0]))*w_m[1][0], (kf*(bl+al)*np.cos(beta[2][0]) + km*np.sin(beta[2][0]))*w_m[2][0], (kf*al*(np.sin(alpha[3][0]))**2*np.cos(beta[3][0]) + kf*(bl+al*np.cos(alpha[3][0]))*np.cos(alpha[3][0])*np.cos(beta[3][0]) - km*np.sin(beta[3][0]))*w_m[3][0]]
            ])
            J_betadot = np.vstack((top,bottom)) #6*4 matrix
            
            left = JR@J_beta
            right = JR@J_betadot
            
            B = np.hstack((left,right))
            
            #드디어 Control input 계산 명령을 주는건데 state class의 변수를 변화시키는 것이 맞나?
            B_pinv = np.linalg.pinv(B)
            inner = v_lqr - JRdot @ J_beta @ w_m
            u_control = B_pinv@inner
            
            
            #Control input 만듬 state class에 있는 것이 맞을지 검토해봐야 할 듯, dt가 처음엔 0일텐데 흠 -> 프로펠러 안돌텐데 그 후엔 dt바뀌니까 되겠네
            
            self.state.w_m = w_m + u_control[0:4]*dt
            F = F_a_b @ self.state.w_m
            Tau = Tau_a_b @ self.state.w_m
            self.F   = F.reshape((3, 1))
            self.Tau = Tau.reshape((3, 1))
            self.state.w_d = np.sqrt(np.maximum(self.state.w_m, 0.0))
            self.state.w_d[0][0] *= -1
            self.state.w_d[2][0] *= -1
            # self.state.w_d[[0,2]] = self.state.w_d[[2,0]]
            # self.state.w_d[[1,3]] = self.state.w_d[[3,1]]
            
            self.state.alpha = np.array([
                [0,0,0,0]
            ]).T # 팔 각도 고정
            self.state.beta_dot = u_control[4:]
            self.state.beta = self.state.beta + self.state.beta_dot * 0.01
            
            # self.state.beta_dot[[0,2]] = self.state.beta_dot[[2,0]]
            # self.state.beta_dot[[1,3]] = self.state.beta_dot[[3,1]]
            
            #F_ab,Tau_ab 업데이트
            self.drone_model.F_ab = F_a_b
            # self.drone_model.F_ab[:,[0,2]]=self.drone_model.F_ab[:,[2,0]]
            # self.drone_model.F_ab[:,[1,3]]=self.drone_model.F_ab[:,[3,1]]
            self.drone_model.Tau_ab = Tau_a_b
            
            
            # self.drone_model.Tau_ab[:,[0,2]]=self.drone_model.Tau_ab[:,[2,0]]
            # self.drone_model.Tau_ab[:,[1,3]]=self.drone_model.Tau_ab[:,[3,1]]
            
            
        elif self.state.mode == 'Y':
            #Z-domain에서 v 값 계산
            
            z_current = np.array([
                x,x_dot,x_ddot,y,y_dot,y_ddot,z,z_dot,z_ddot,phi,phi_dot,phi_ddot,theta,theta_dot,theta_ddot,psi, psi_dot, psi_ddot
            ]).T
            z_ref = np.array([
                x_d,0,0,y_d,0,0,z_d,0,0,phi_d,0,0,theta_d,0,0,psi_d,0,0
            ]).T
            error = z_ref-z_current
                    #이런식으로 K_lqr 계산, 미리 offline 계산
           
            K_lqr = np.array([
                [ 1.0000,  5.5826,  5.5826,  0.0000, -0.0000,  0.0000,  0.0000,  0.0000,  0.0000, -0.0000, -0.0000,  0.0000, -0.0000, -0.0000, -0.0000, -0.0000,  0.0000,  0.0000],
                [ 0.0000, -0.0000,  0.0000,  1.0000,  5.5826,  5.5826,  0.0000,  0.0000,  0.0000, -0.0000,  0.0000,  0.0000, -0.0000, -0.0000, -0.0000, -0.0000, -0.0000,  0.0000],
                [ 0.0000, -0.0000,  0.0000, -0.0000, -0.0000, -0.0000,  1.0000,  5.5826,  5.5826,  0.0000,  0.0000,  0.0000,  0.0000, -0.0000,  0.0000,  0.0000,  0.0000, -0.0000],
                [ 0.0000,  0.0000,  0.0000, -0.0000, -0.0000, -0.0000,  0.0000,  0.0000,  0.0000,  10.0000,  10.5826,  10.5826,  0.0000,  0.0000,  0.0000, -0.0000, -0.0000, -0.0000],
                [-0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000, -0.0000, -0.0000, -0.0000, -0.0000,  0.0000,  0.0000,  10.0000,  10.5826,  10.5826, -0.0000,  0.0000,  0.0000],
                [-0.0000,  0.0000, -0.0000,  0.0000,  0.0000,  0.0000, -0.0000, -0.0000,  0.0000,  0.0000,  0.0000, -0.0000,  0.0000, -0.0000, -0.0000,  10.0000,  10.5826,  10.5826]
            ])
            
            v_lqr = (K_lqr@error).reshape((6,1))
            self.state.v = v_lqr
            #Y configuration alpha_dot 논리, alpha 명령 줌 alpha_dot 아님
            failnum = self.fault_detection.failnum
            i = failnum
            
            if failnum == 1:
                
                self.state.alpha = np.zeros((4,1))
                self.state.alpha[1,0]= -np.pi/6
                self.state.alpha[3,0]=  np.pi/6
            elif failnum == 2:

                self.state.alpha = np.zeros((4,1))
                self.state.alpha[2,0]= np.pi/6
                self.state.alpha[0,0]= -np.pi/6
            elif failnum == 3:
                self.state.alpha = np.zeros((4,1))
                self.state.alpha[3,0]= np.pi/6
                self.state.alpha[1,0]= -np.pi/6
            elif failnum == 4: 
                self.state.alpha = np.zeros((4,1))
                self.state.alpha[0,0]= np.pi/6
                self.state.alpha[2,0]= -np.pi/6

            
            #CM 위치 계산
            p2x = np.array([
                [np.cos(-np.pi/4), -np.sin(-np.pi/4),0],
                [np.sin(-np.pi/4), np.cos(-np.pi/4),0],
                [0,0,1]
                ])
            
            acml = self.drone_model.acml
            m_a = self.drone_model.m_a
            cmArm1 = np.array([[ acml*np.cos(alpha[0][0])+bl ],
                                [ acml*np.sin(alpha[0][0])     ],
                                [ 0                          ]])
            cmArm2 = np.array([[ -acml*np.sin(alpha[1][0])             ],
                                [  acml*np.cos(alpha[1][0])+bl         ],
                                [  0                                 ]])
            cmArm3 = np.array([[ -acml*np.cos(alpha[2][0]) - bl         ],
                                [ -acml*np.sin(alpha[2][0])             ],
                                [  0                                 ]])
            cmArm4 = np.array([[  acml*np.sin(alpha[3][0])              ],
                                [ -acml*np.cos(alpha[3][0]) - bl         ],
                                [  0                                 ]])
            cmTot =  ( (m_a * cmArm1 + m_a * cmArm2 + m_a * cmArm3 + m_a * cmArm4) / m_t )

            print("cmTot", cmTot)
            #I_d 값 계산(Numerical method)
            I_inv_cur = np.linalg.inv(I_cur)
            I_inv_prev = np.linalg.inv(I_prev)
            I_d = (I_inv_cur-I_inv_prev)/dt
            print("alpha", alpha[1][0], alpha[3][0])
            #F_a_b 값 계산
            F_a_b =  p2x @ np.array([
                [0, kf*np.sin(beta[0][0]), -kf*np.cos(beta[0][0])],
                [-kf*np.cos(alpha[1][0])*np.sin(beta[1][0]), -kf*np.sin(alpha[1][0])*np.sin(beta[1][0]), -kf*np.cos(beta[1][0])],
                [0, -kf*np.sin(beta[2][0]), -kf*np.cos(beta[2][0])],
                [kf*np.cos(alpha[3][0])*np.sin(beta[3][0]), kf*np.sin(alpha[3][0])*np.sin(beta[3][0]), -kf*np.cos(beta[3][0])]
            ]).T
            
            #Tau_a_b 값 계산 #bl에 1/sqrt(2) 안곱하는 이유 궁금
            lcm = cmTot[0][0]
            
            Tau_a_b =  p2x @ np.array([
                            [0,                                                                                                        kf*(bl+al-lcm),                                                                                 -km],
                            [-kf*(bl+al*np.cos(alpha[1][0]))*np.cos(beta[1][0]) + km*np.cos(alpha[1][0])*np.sin(beta[1][0]),     -kf*(al*np.sin(alpha[1][0])+lcm)*np.cos(beta[1][0])+km*np.sin(alpha[1][0])*np.sin(beta[1][0]),            kf*(al*np.sin(alpha[1][0])+lcm)*np.sin(alpha[1][0])*np.sin(beta[1][0])+kf*(bl+al*np.cos(alpha[1][0]))*np.cos(alpha[1][0])*np.sin(beta[1][0])+ km*np.cos(beta[1][0])],
                            [                                       0,                                                          -kf*(bl+al+lcm)*np.cos(beta[2][0]) - km*np.sin(beta[2][0]),                                       kf*(bl+al+lcm)*np.sin(beta[2][0]) - km*np.cos(beta[2][0])],
                            [kf*(bl+al*np.cos(alpha[3][0]))*np.cos(beta[3][0])-km*np.cos(alpha[3][0])*np.sin(beta[3][0]),      kf*(al*np.sin(alpha[3][0])-lcm)*np.cos(beta[3][0])-km*np.sin(alpha[3][0])*np.sin(beta[3][0]),            kf*(al*np.sin(alpha[3][0])-lcm)*np.sin(alpha[3][0])*np.sin(beta[3][0])+kf*(bl+al*np.cos(alpha[3][0]))*np.cos(alpha[3][0])*np.sin(beta[3][0])+km*np.cos(beta[3][0])]
                        ]).T
            
            #JR matrix 6*6
            R = rpy2rot(phi,theta,psi)         # 3×3 회전 행렬
            R_T = R.T                      # RPY2Rot(obj.euler)'에 해당

            top_left = (1 / m_t) * R_T
            top_right = np.zeros((3, 3))
            bottom_left = np.zeros((3, 3))
            bottom_right = np.linalg.inv(I_cur)
            
            JR = np.block([
                [top_left,     top_right],
                [bottom_left,  bottom_right]
            ])
            
            #JR_dot matrix 
            R = RPY2Rot_derivative(phi,theta,psi,phi_dot,theta_dot,psi_dot)
            R_T = R.T
            top_left = (1/m_t)*R_T
            top_right = np.zeros((3,3))
            bottom_left = np.zeros((3,3))
            bottom_right = np.zeros((3,3))
            
            
            JRdot = np.block([
                [top_left, top_right],
                [bottom_left,bottom_right]
            ])
            
            #J_beta, J_betadot 계산후 B matrix 
            J_beta = np.vstack((F_a_b,Tau_a_b)) #6*4
            
            top = p2x @ np.array([
                [0,  -kf*np.cos(alpha[1][0])*np.cos(beta[1][0])*w_m[1][0], 0, kf*np.cos(alpha[3][0])*np.cos(beta[3][0])*w_m[3][0]],
                [kf*np.cos(beta[0][0])*w_m[0][0],  -kf*np.sin(alpha[1][0])*np.cos(beta[1][0])*w_m[1][0], -kf*np.cos(beta[2][0])*w_m[2][0], kf*np.sin(alpha[3][0])*np.cos(beta[3][0])*w_m[3][0]],
                [kf*np.sin(beta[0][0])*w_m[0][0],kf*np.sin(beta[1][0])*w_m[1][0], kf*np.sin(beta[2][0])*w_m[2][0],kf*np.sin(beta[3][0])*w_m[3][0]]
            ])
            # 하단 블록 수정본: lcm 포함
            bottom = p2x @ np.array([
                [
                # Row 1 unchanged
                0,
                kf*(bl+al*np.cos(alpha[1][0]))*np.sin(beta[1][0])*w_m[1][0] + \
                    km*np.cos(alpha[1][0])*np.cos(beta[1][0])*w_m[1][0],
                0,
                -kf*(bl+al*np.cos(alpha[3][0]))*np.sin(beta[3][0])*w_m[3][0] - \
                    km*np.cos(alpha[3][0])*np.cos(beta[3][0])*w_m[3][0]
                ],
                [
                # Row 2: include +lcm term
                (-kf*(bl+al)*np.sin(beta[0][0]) + km*np.cos(beta[0][0]))*w_m[0][0],
                (kf*(al*np.sin(alpha[1][0]) + lcm)*np.sin(beta[1][0]) + \
                    km*np.sin(alpha[1][0])*np.cos(beta[1][0]))*w_m[1][0],
                (kf*(bl+al)*np.sin(beta[2][0]) - km*np.cos(beta[2][0]))*w_m[2][0],
                -(kf*(al*np.sin(alpha[3][0]) - lcm)*np.sin(beta[3][0]) + \
                    km*np.sin(alpha[3][0])*np.cos(beta[3][0]))*w_m[3][0]
                ],
                [
                # Row 3: include +lcm in both terms
                (kf*(bl+al)*np.cos(beta[0][0]) + km*np.sin(beta[0][0]))*w_m[0][0],
                (kf*(al*np.sin(alpha[1][0]) + lcm)**2 * np.cos(beta[1][0]) + \
                    kf*(bl+al*np.cos(alpha[1][0]))*np.cos(alpha[1][0])*np.cos(beta[1][0]) - \
                    km*np.sin(beta[1][0]))*w_m[1][0],
                (kf*(bl+al+lcm)*np.cos(beta[2][0]) + km*np.sin(beta[2][0]))*w_m[2][0],
                (kf*(al*np.sin(alpha[3][0]) - lcm)**2 * np.cos(beta[3][0]) + \
                    kf*(bl+al*np.cos(alpha[3][0]))*np.cos(alpha[3][0])*np.cos(beta[3][0]) - \
                    km*np.sin(beta[3][0]))*w_m[3][0]
                ]
            ])

            J_betadot = np.vstack((top,bottom)) #6*4
            
            left = JR@J_beta
            right = JR@J_betadot
            
            B = np.hstack((left,right)) #6*8
            print("B", B)
            #fail 난 부분 삭제
            
            B = np.delete(B,[i-1,i+3],axis =1)
            print("B", B)
            #드디어 Control input 계산 명령을 주는건데 state class의 변수를 변화시키는 것이 맞나?
            # B_inv = np.linalg.inv(B)
            
            
            J_beta = np.delete(J_beta,i-1,axis = 1)
            


            w_m = np.delete(w_m,i-1,axis = 0)
            
            B_pinv = np.linalg.pinv(B)
            inner = v_lqr - JRdot @ J_beta @ w_m
            u_control = B_pinv@inner
            
            
            ##Control input 만듬 state class에 있는 것이 맞을지 검토해봐야 할 듯
            w_m = np.insert(w_m,i-1,np.zeros((1,1)),axis=0)
            
            self.state.w_m = w_m + np.insert(u_control[0:3,:],i-1,np.zeros((1,1)),axis=0)*0.01
            self.state.beta_dot = np.insert(u_control[3:6,:],i-1,np.zeros((1,1)),axis=0)
           
            
            print("F_a_b", F_a_b)
            print("Tau_a_b", Tau_a_b)
            print("w_m", self.state.w_m)
            Force = F_a_b @ self.state.w_m
            Moment = Tau_a_b @ self.state.w_m
            self.F   = Force.reshape((3, 1))
            self.Tau = Moment.reshape((3, 1))
            print("F", self.F)
            print("Tau",self.Tau)
            self.state.w_d = np.sqrt(np.maximum(self.state.w_m, 0.0))
            self.state.w_d[0][0] *= -1
            self.state.w_d[2][0] *= -1
            # self.state.w_d[[0,2]] = self.state.w_d[[2,0]]
            # self.state.w_d[[1,3]] = self.state.w_d[[3,1]]
            
            
            
            self.state.beta = self.state.beta + self.state.beta_dot * 0.01
            
            #F_ab Tau_ab 업데이트
            self.drone_model.F_ab = F_a_b
            self.drone_model.Tau_ab = Tau_a_b
        