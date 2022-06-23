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
        self.motor_trust = np.array([[0.0], [0.0], [0.0]])
        self.motor_moments = np.array([[0.0], [0.0], [0.0]])

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

        def update_state(self, u , dt):
            self._func_rigth(u)
            self._integrate(dt)

        def _integrate(self, dt):
            self.state_vector[6:9] += self._linear_acceleration * dt
            self.state_vector[0:3] += self.state_vector[6:9] * dt
            self.state_vector[9:12] += self.angular_acceleration * dt
            self.state_vector[3:6] += self.state_vector[9:12] * dt

        def _func_right(self, u):
