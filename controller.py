import numpy as np
import costants as cs
from costants import FlightMode, States

class PID:
    def __init__(self):
        self._k_p = 1
        self._k_i = 0
        self._k_d = 0.1
        self._integral = 0
        self._last_error = 0
        self._integral_limit = 300
        self._min_value = -1
        self._max_value = 1

    def set_pid_gains(self, k_p, k_i, k_d):
        self._k_p = k_p
        self._k_i = k_i
        self._k_d = k_d

    def set_integral_limit(self, limit):
        self._integral_limit = limit

    def set_saturation_limit(self, min, max):
        self._min_value = min
        self._max_value = max

    def update(self, input_val, target_val, dt):
        error = target_val - input_val
        self._integral += error * dt
        self._integral = self.saturation(self._integral, -self._integral_limit, self._integral_limit)

        P = self._k_p * error
        I = self._k_i * error
        D = self._k_d * error

        self._last_error = error
        output = P + I + D
        output = self.saturation(output, self._min_value, self._max_value)
        return output

    def saturation(self, input, min, max):
        if input > max:
            return max
        elif input < min:
            return min
        else:
            return input


class QuadcopterController:
    def __init__(self):
        # self.target_x = 0
        # self.target_y = 0
        # self.target_z = 0
        # self.target_yaw = 0

        self._u = np.array([[0.0], [0.0], [0.0], [0.0]])

        self.position_controller_x = PID()
        self.position_controller_y = PID()
        self.position_controller_z = PID()

        self.velocity_controller_x = PID()
        self.velocity_controller_y = PID()
        self.velocity_controller_z = PID()

        self.roll_controller = PID()
        self.pitch_controller = PID()
        self.yaw_controller = PID()

        self.roll_rate_controller = PID()
        self.pitch_rate_controller = PID()
        self.yaw_rate_controller = PID()

    def set_target_position(self, x=0, y=0, z=0, yaw=0):
        self.target_x = x
        self.target_y = y
        self.target_z = z
        self.target_yaw = yaw
        # return self.target_x, self.target_y, self.target_z, self.target_yaw

        # self.target_x, self.target_y, self.target_z, self.target_yaw = set_target_position()

    def update(self, state_vector, dt):

        target_vel_x = self.position_controller_x.update(state_vector[States.X], self.target_x, dt)
        target_vel_y = self.position_controller_y.update(state_vector[States.Y], self.target_y, dt)
        target_vel_z = self.position_controller_z.update(state_vector[States.Z], self.target_z, dt)

        target_roll = self.velocity_controller_x.update(state_vector[States.VX], target_vel_x, dt)
        target_pitch = self.velocity_controller_y.update(state_vector[States.VY], target_vel_y, dt)
        cmd_trust = self.velocity_controller_z.update(state_vector[States.VZ], target_vel_z, dt)

        cmd_trust *= cs.trust_scale
        cmd_trust = self.velocity_controller_z.saturation(cmd_trust, cs.min_rotors_rpm, cs.max_rotors_rpm)

        target_pitch_roll = self._rotation2d(state_vector[States.YAW][0]).transpose() @ np.array([[target_pitch][0], [target_roll][0]])

        target_roll = target_pitch_roll[0]
        target_pitch = target_pitch_roll[1]

        target_roll_rate = self.roll_controller.update(state_vector[States.ROLL], -target_roll, dt)
        target_pitch_rate = self.pitch_controller.update(state_vector[States.PITCH], target_pitch, dt)
        target_yaw_rate = self.yaw_rate_controller.update(state_vector[States.YAW], self.target_yaw, dt)

        cmd_roll = self.roll_rate_controller.update(state_vector[States.ROLL_RATE], target_roll_rate, dt)
        cmd_pitch = self.pitch_rate_controller.update(state_vector[States.PITCH_RATE], target_pitch_rate, dt)
        cmd_yaw = self.yaw_rate_controller.update(state_vector[States.YAW_RATE], target_yaw_rate, dt)

        self._mixer(cmd_trust, cmd_roll, cmd_pitch, cmd_yaw)
        return self._u

    def _mixer(self, cmd_trust, cmd_roll, cmd_pitch, cmd_yaw):
        self._u[0] = cmd_trust + cmd_roll - cmd_yaw
        self._u[1] = cmd_trust - cmd_pitch + cmd_yaw
        self._u[2] = cmd_trust - cmd_roll - cmd_yaw
        self._u[3] = cmd_trust + cmd_pitch + cmd_yaw

    def _rotation2d(self, theta):
        return np.array([[np.cos(theta), -np.sin(theta)],
                         [np.sin(theta), np.cos(theta)]])


