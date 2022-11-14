import socket
import struct
import time as tm

class Simulator:

    def __init__(self, controller, model, dt, target_points):
        self.controller = controller
        self.model = model
        self.dt = dt
        self.target_points = target_points
        self.time = 0
        self._host = '127.0.0.1'
        self._port = 12345
        self.addr = (self._host, self._port)
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.count_point = 0
        self.target_x = self.target_points[self.count_point][0]
        self.target_y = self.target_points[self.count_point][1]
        self.target_z = self.target_points[self.count_point][2]

    def run(self):
        while True:
            state = self.model.get_state_vector()
            # self.controller.set_target_position(0, 0, 0)
            if (abs(state[0] - self.target_points[self.count_point][0]) < 0.5) \
                    and (abs(state[1] - self.target_points[self.count_point][1]) < 0.5) \
                    and (abs(state[2] - self.target_points[self.count_point][2]) < 0.5) \
                    and (len(self.target_points)-1 > self.count_point):
                print('next point')

                self.target_x, self.target_y, self.target_z = self.target_points[self.count_point][0], self.target_points[self.count_point][1], self.target_points[self.count_point][2]

                self.count_point += 1
            self.controller.set_target_position(self.target_x, self.target_y, self.target_z)
            u = self.controller.update(state, self.dt)
            self.model.update_state(u, self.dt)
            self._send_pose_data(state)
            self.time += self.dt
            tm.sleep(self.dt)

    def _send_pose_data(self, state):
        # state[0] = 0
        # state[1] = 0
        # state[2] = 0
        data = bytearray(struct.pack("ddddddddddddd", state[0], state[1], state[2], state[3], state[4], state[5], state[6], state[7], state[8], state[9], state[10], state[11], self.dt))
        self.udp_socket.sendto(data, self.addr)
        # print(data)

