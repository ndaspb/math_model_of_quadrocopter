import numpy as np


class QuadcopterModel:
    def __init__(self, inertia, mass, trust_coeff, drag_coeff, arm_length):
        self.inertia = inertia
        self._inertia_inv = np.linalg.inv(inertia)
        self.mass = mass
        self.trust_coeff = trust_coeff
        self.drag_coeff = drag_coeff
        self.arm_length = arm_length
        self._g = np.array([[0.0], [0.0], [-9.81]])
        self._motor_trust = np.array([[0.0], [0.0], [0.0]])
        self._motor_moments = np.array([[0.0], [0.0], [0.0]])

        self.state_vector = np.array([[0.0],  # pos X
                                      [0.0],  # pos Y
                                      [0.0],  # pos Z
                                      [0.0],  # roll
                                      [0.0],  # pitch
                                      [0.0],  # yaw
                                      [0.0],  # vel X
                                      [0.0],  # vel Y
                                      [0.0],  # vel Z
                                      [0.0],  # roll rate
                                      [0.0],  # pitch rate
                                      [0.0]])  # yaw rate

    def get_state_vector(self):
        return self.state_vector

    def update_state(self, u, dt):
        self._func_right(u)
        self._integrate(dt)

    def _integrate(self, dt):
        self.state_vector[6:9] += self._linear_acceleration * dt
        self.state_vector[0:3] += self.state_vector[6:9] * dt
        self.state_vector[9:12] += self._angular_acceleration * dt
        self.state_vector[3:6] += self.state_vector[9:12] * dt

    def _func_right(self, u):
        self._motor_trust[2] = (u**2 * self.trust_coeff).sum()
        R = self._rotation_matrix_3d(self.state_vector[3], self.state_vector[4], self.state_vector[5])
        R.resize((3,3))
        self._linear_acceleration = R.transpose() @ self._motor_trust / self.mass + self._g
        self._motor_moments[0] = self.arm_length * self.trust_coeff * (u[0]**2-u[2]**2)
        self._motor_moments[1] = self.arm_length * self.trust_coeff * (u[3] ** 2 - u[1] ** 2)
        self._motor_moments[2] = self.drag_coeff * (u[3]**2 + u[1]**2 - u[0]**2 - u[2]**2)
        self._angular_acceleration = self._inertia_inv @ (self._motor_moments - np.cross((self._motor_moments - self.state_vector[9:12]), (self.inertia @ self.state_vector[9:12]), axis=0))

    def _rotation_matrix_3d(self, pitch, roll, yaw):
        return np.array([[np.cos(yaw)*np.cos(roll), np.sin(yaw)*np.cos(roll), -np.sin(roll)],
                            [np.cos(yaw)*np.sin(pitch)*np.sin(roll) - np.sin(yaw)*np.cos(pitch),
                            np.sin(yaw)*np.sin(pitch)*np.sin(roll) + np.cos(yaw)*np.cos(pitch),
                            np.sin(pitch)*np.cos(roll)],
                            [np.cos(yaw)*np.sin(roll)*np.cos(pitch) + np.sin(yaw)*np.sin(pitch),
                            np.sin(yaw)*np.cos(pitch)*np.sin(roll) - np.cos(yaw)*np.sin(pitch),
                            np.cos(pitch)*np.cos(roll)]])

