
import numpy as np
from scipy.spatial.transform import Rotation


class SE3Control(object):
    """

    """

    def __init__(self, quad_params):
        """
        This is the constructor for the SE3Control object. You may instead
        initialize any parameters, control gain values, or private state here.

        For grading purposes the controller is always initialized with one input
        argument: the quadrotor's physical parameters. If you add any additional
        input arguments for testing purposes, you must provide good default
        values!

        Parameters:
            quad_params, dict with keys specified by crazyflie_params.py

        """

        # Quadrotor physical parameters.
        self.mass = quad_params['mass']  # kg
        self.Ixx = quad_params['Ixx']  # kg*m^2
        self.Iyy = quad_params['Iyy']  # kg*m^2
        self.Izz = quad_params['Izz']  # kg*m^2
        self.arm_length = quad_params['arm_length']  # meters
        self.rotor_speed_min = quad_params['rotor_speed_min']  # rad/s
        self.rotor_speed_max = quad_params['rotor_speed_max']  # rad/s
        self.k_thrust = quad_params['k_thrust']  # N/(rad/s)**2
        self.k_drag = quad_params['k_drag']  # Nm/(rad/s)**2

        # Additional constants including control gains.
        self.inertia = np.diag(np.array([self.Ixx, self.Iyy, self.Izz]))  # kg*m^2
        self.g = 9.81  # m/s^2

        # self.Kd = np.array([[8.5, 0, 0], [0, 8.5, 0], [0, 0, 8.5]])
        # self.Kp = np.array([[5, 0, 0], [0, 5, 0], [0, 0, 5]])
        # self.Kr = np.array([[250, 0, 0], [0, 250, 0], [0, 0, 250]])
        # self.Kw = np.array([[35, 0, 0], [0, 35, 0], [0, 0, 35]])

        # self.Kp = np.array([[8.5, 0, 0], [0, 8.5, 0], [0, 0, 8.5]])
        # self.Kd = np.array([[8.5, 0, 0], [0, 8.5, 0], [0, 0, 8]])
        # self.Kr = np.array([[250, 0, 0], [0, 250, 0], [0, 0, 300]])
        # self.Kw = np.array([[35, 0, 0], [0, 35, 0], [0, 0, 45]])


        # self.Kd = np.array([[4,0,0],[0,4,0],[0,0,5]])
        # self.Kp = np.array([[3,0,0],[0,3,0],[0,0,5]])
        # self.Kr = np.array([[40,0,0],[0,40,0],[0,0,40]])
        # self.Kw = np.array([[6,0,0],[0,6,0],[0,0,20]])

        # self.Kd = np.array([[4,0,0],[0,4,0],[0,0,5]])
        # self.Kp = np.array([[3,0,0],[0,3,0],[0,0,5]])
        # self.Kr = np.array([[250,0,0],[0,400,0],[0,0,250]])
        # self.Kw = np.array([[10,0,0],[0,30,0],[0,0,10]])

        ####Previous##########
        # self.Kp = np.diag(np.array([8, 8, 8]))
        # self.Kd = np.diag(np.array([4.45, 4.45, 4.45]))
        # self.Kr = np.diag(np.array([250, 250, 300]))
        # self.Kw = np.diag(np.array([35, 35, 45]))
        ######################

###############________BEST________#################
        self.Kp = np.diag(np.array([7, 7, 15]))
        self.Kd = np.diag(np.array([4.4, 4.4, 7]))
        self.Kr = np.diag(np.array([2600, 2600, 150]))
        self.Kw = np.diag(np.array([130, 130, 80]))
####################################################

        # self.Kp = np.diag(np.array([7, 7, 15]))
        # self.Kd = np.diag(np.array([4.4, 4.4, 7]))
        # self.Kr = np.diag(np.array([4000, 4000, 300]))
        # self.Kw = np.diag(np.array([130, 130, 80]))

        # self.Kp = np.diag(np.array([13, 13, 18]))
        # self.Kd = np.diag(np.array([7.75, 7.75, 7]))
        # self.Kr = np.diag(np.array([4000, 4000, 300]))
        # self.Kw = np.diag(np.array([170, 170, 325]))

        # self.Kp = np.diag(np.array([10, 10, 10]))
        # self.Kd = np.diag(np.array([5.45, 5.45, 5.45]))
        # self.Kr = np.diag(np.array([250, 250, 300]))
        # self.Kw = np.diag(np.array([35, 35, 45]))

        # self.Kd = np.array([[4, 0, 0], [0, 4, 0], [0, 0, 5]])
        # self.Kp = np.array([[3, 0, 0], [0, 3, 0], [0, 0, 7]])
        # self.Kr = np.array([[40, 0, 0], [0, 40, 0], [0, 0, 40]])
        # self.Kw = np.array([[6, 0, 0], [0, 6, 0], [0, 0, 20]])

        # self.Kd = np.array([[5, 0, 0], [0, 5, 0], [0, 0, 5.5]])
        # self.Kp = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 4]])
        # self.Kr = np.array([[40, 0, 0], [0, 40, 0], [0, 0, 40]])
        # self.Kw = np.array([[6, 0, 0], [0, 6, 0], [0, 0, 20]])

        # self.Kd = np.array([[7, 0, 0], [0, 4, 0], [0, 0, 7]])
        # self.Kp = np.array([[3, 0, 0], [0, 3, 0], [0, 0, 7]])
        # self.Kr = np.array([[40, 0, 0], [0,45, 0], [0, 0, 40]])
        # self.Kw = np.array([[5, 0, 0], [0, 6, 0], [0, 0, 25]])
        # self.w_des = 0

    def update(self, t, state, flat_output):
        """
        This function receives the current time, true state, and desired flat
        outputs. It returns the command inputs.

        Inputs:
            t, present time in seconds
            state, a dict describing the present state with keys
                x, position, m
                v, linear velocity, m/s
                q, quaternion [i,j,k,w]
                w, angular velocity, rad/s
            flat_output, a dict describing the present desired flat outputs with keys
                x,        position, m
                x_dot,    velocity, m/s
                x_ddot,   acceleration, m/s**2
                x_dddot,  jerk, m/s**3
                x_ddddot, snap, m/s**4
                yaw,      yaw angle, rad
                yaw_dot,  yaw rate, rad/s

        Outputs:
            control_input, a dict describing the present computed control inputs with keys
                cmd_motor_speeds, rad/s
                cmd_thrust, N (for debugging and laboratory; not used by simulator)
                cmd_moment, N*m (for debugging; not used by simulator)
                cmd_q, quaternion [i,j,k,w] (for laboratory; not used by simulator)
        """
        cmd_motor_speeds = np.zeros((4,))
        cmd_thrust = 0
        cmd_moment = np.zeros((3,))
        cmd_q = np.zeros((4,))

        x_ddot_des = flat_output['x_ddot'].reshape(3, 1) - (
                    self.Kd @ (state['v'] - flat_output['x_dot']).reshape(3, 1)) - (
                                 self.Kp @ (state['x'] - flat_output['x']).reshape(3, 1))
        F_des = (self.mass * x_ddot_des) + np.array([0, 0, self.mass * self.g]).reshape(3, 1)
        R = Rotation.from_quat(state['q']).as_matrix()
        b_3 = R @ np.array([0, 0, 1]).reshape(3, 1)
        u_1 = (b_3.T @ F_des).flatten()
        # print("u_1", "\n", u_1)

        b3_des = F_des / np.linalg.norm(F_des)
        # print("b3_des", "\n", b3_des)
        yaw_head = np.array([np.cos(flat_output['yaw']), np.sin(flat_output['yaw']), 0]).reshape(3, 1)
        b2_des = np.cross(b3_des.T, yaw_head.T) / np.linalg.norm(np.cross(b3_des.T, yaw_head.T))
        b2_des = b2_des.reshape(3, 1)
        # print("b2_des", "\n", b2_des)
        R_des = np.zeros((3, 3))
       
        # print("sim", "\n", sim)
        R_des[:, 0] = np.cross(b2_des.T, b3_des.T)
        R_des[:, 1] = b2_des.T
        R_des[:, 2] = b3_des.T
        

        eR_temp = (R_des.T @ R) - (R.T @ R_des)
        eR = np.array([eR_temp[2, 1], eR_temp[0, 2], eR_temp[1, 0]]) / 2
        eR = eR.reshape(3, 1)
        eW = state['w']
        eW = eW.reshape(3, 1)
        u_2 = self.inertia @ (-(self.Kr @ eR) - (self.Kw @ eW))

        cmd_thrust = u_1
        cmd_moment = u_2
        u = np.append(u_1, u_2).reshape(4, 1)
        # print("u_2", "\n", u_2)

        L = self.arm_length
        gamma = self.k_drag / self.k_thrust
        mat_temp = np.array([[1, 1, 1, 1], [0, L, 0, -L], [-L, 0, L, 0], [gamma, -gamma, gamma, -gamma]])
        motor_thrusts = np.linalg.inv(mat_temp) @ u / self.k_thrust
        # print("motor_thrusts", "\n", motor_thrusts)
        cmd_motor_speeds = np.sqrt(np.clip(motor_thrusts, self.rotor_speed_min ** 2, self.rotor_speed_max ** 2))
        cmd_q = Rotation.from_matrix(R_des).as_quat()

        control_input = {'cmd_motor_speeds': cmd_motor_speeds,
                         'cmd_thrust': cmd_thrust,
                         'cmd_moment': cmd_moment,
                         'cmd_q': cmd_q}
        return control_input
