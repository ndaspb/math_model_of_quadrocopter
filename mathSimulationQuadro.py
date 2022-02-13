import yaml
import numpy as np
from matplotlib import pyplot as plt


def read_yaml(t):
    """
    Read parameters of quad-rotor in Yaml file

    :param t: string # name of yaml-file
    :return: list
    """
    with open(t, encoding="utf8") as fh:
        read_data = yaml.load(fh, Loader=yaml.FullLoader)
    s = []
    for key, value in read_data.items():
        s.append(value)
    return s


class ControlSystemQuadro:
    def __init__(self, x, y, z):

        self.acceleration = np.array([[0], [0], [0]])
        self.velocity = np.array([[0], [0], [0]])
        self.position = np.array([[0], [0], [0]])
        self.angularAcceleration = np.array([[0], [0], [0]])
        self.angularVelocity = np.array([[0], [0], [0]])
        self.orientation = np.array([[0], [0], [0]])

        self.desired_x = x
        self.desired_y = y
        self.desired_z = z

        self.error_past_z = self.desired_z
        self.current_position_z = self.desired_z
        self.g = 9.81
        self.poseList = []


        # считываем параметры из YAML-файла
        self.model_config = read_yaml('quadModelConfig.yaml')
        self.control_system = read_yaml('quadControlSystem.yaml')
        print(self.model_config)
        print(self.control_system)

        # Устанавливаем параметры математической модели
        self.mass = self.model_config[2]
        self.lengthOfFlyerArms = self.model_config[5]
        self.motorThrustCoef = self.model_config[3]
        self.motorResistCoef = self.model_config[4]
        self.numberOfRotors = self.model_config[6]
        self.Ixx = self.model_config[7]
        self.Iyy = self.model_config[8]
        self.Izz = self.model_config[9]
        self.VelocityRotors = self.model_config[10]

        # Устанавливаем параметры симулятора
        self.dt = self.model_config[0]
        self.simulationTotalTime = self.model_config[1]

        # Устанавливаем параметры системы управления коэфициенты ПИД регулирования
        self.KpXAngularRate = self.control_system[0]
        self.KiXAngularRate = self.control_system[1]
        self.KdXAngularRate = self.control_system[2]

        self.KpYAngularRate = self.control_system[3]
        self.KiYAngularRate = self.control_system[4]
        self.KdYAngularRate = self.control_system[5]

        self.KpZAngularRate = self.control_system[6]
        self.KiZAngularRate = self.control_system[7]
        self.KdZAngularRate = self.control_system[8]

        self.KpXAngle = self.control_system[9]
        self.KiXAngle = self.control_system[10]
        self.KdXAngle = self.control_system[11]

        self.KpYAngle = self.control_system[12]
        self.KiYAngle = self.control_system[13]
        self.KdYAngle = self.control_system[14]

        self.KpZAngle = self.control_system[15]
        self.KiZAngle = self.control_system[16]
        self.KdZAngle = self.control_system[17]

        self.KpXPosition = self.control_system[18]
        self.KiXPosition = self.control_system[19]
        self.KdXPosition = self.control_system[20]

        self.KpYPosition = self.control_system[21]
        self.KiYPosition = self.control_system[22]
        self.KdYPosition = self.control_system[23]

        self.KpZPosition = self.control_system[24]
        self.KiZPosition = self.control_system[25]
        self.KdZPosition = self.control_system[26]

        # Ограничение значений выхода ПИД-регулятора по угловому положению
        self.maxCommandAngle = self.control_system[27]
        # Ограничения максимального значения выхода ПИД-регулятора по угловой скорости
        self.maxCommandAngularRate = self.control_system[28]
        # Ограничения максимального значения выхода ПИД-регулятора по угловому ускорению
        self.maxCommandAngularAcceleration = self.control_system[29]
        # Ограничение по тяге
        self.maxThrust = self.control_system[30]

    # def printing(self):
    #     print(self.motorThrustCoef, self.maxThrust)

    def right_parts(self, rotorAngularVelocity): # pastStateVector, rotorAngularVelocity
        inertialTensor = np.array([[self.Ixx, 0, 0],
                                  [0, self.Iyy, 0],
                                  [0, 0, self.Izz]])

        sumRotorsVelocity = 0

        # momentThrustRotors = np.array([0, 0, 0])
        # print(momentThrustRotors)

        normalizeVector = np.array([0, 0, 1])

        for i in range(self.numberOfRotors):
            sumRotorsVelocity += rotorAngularVelocity[i]**2

        momentThrustRotors = np.array([[self.lengthOfFlyerArms * self.motorThrustCoef *
                                        (rotorAngularVelocity[0]**2 - rotorAngularVelocity[2]**2)],
                                    [self.lengthOfFlyerArms * self.motorThrustCoef * (rotorAngularVelocity[3]**2 -
                                                                                   rotorAngularVelocity[1]**2)],
                                    [self.motorResistCoef * (rotorAngularVelocity[3]**2 + rotorAngularVelocity[1]**2
                                                          - rotorAngularVelocity[0]**2 - rotorAngularVelocity[2]**2)]])

        print(momentThrustRotors)
        print(rotorAngularVelocity)
        print(sumRotorsVelocity)


        # print(inertialTensor)
        thrust_of_rotors = (self.numberOfRotors * (self.VelocityRotors**2)) * self.motorThrustCoef
        # print(thrust_of_rotors)
        acceleration = thrust_of_rotors / self.mass - self.g
        velocity = acceleration * self.dt
        position = velocity * self.dt

        return position



    def desired_position(self):
        desired_position = np.array([self.desired_x, self.desired_y, self.desired_z])

        return desired_position
        # angularRateError =

    def desired_z_position(self):
        error_z = self.desired_z - self.current_position_z
        # print(error_z)
        error_integral_position_z = self.current_position_z
        error_integral_position_z += error_z * self.dt
        p_des_z = self.KpZPosition * error_z + \
                self.KiZPosition * error_integral_position_z + \
                self.KdZPosition * ((error_z - self.error_past_z)/self.dt)
        self.error_past_z = error_z

        if p_des_z > self.VelocityRotors:
            p_des_z = self.VelocityRotors
        elif p_des_z < - self.VelocityRotors:
            p_des_z = - self.VelocityRotors
        acceleration = (self.motorThrustCoef * (p_des_z**2) * self.numberOfRotors) / self.mass - self.g
        velocity = acceleration * self.dt
        position = velocity * self.dt

        return position


    def run_simulator(self):
        time = 0

        while time < 5: # self.simulationTotalTime:
            pose = self.desired_z_position()
            self.poseList.append(pose)
            time += self.dt

xoxo = ControlSystemQuadro(0, 0, 10)
# posez = xoxo.run_simulator()
xoxo.right_parts(np.array([100, 100, 300, 100]))

# def showPlots():
#
#     f = plt.figure(constrained_layout=True)
#     gs = f.add_gridspec(3, 5)
#     ax1 = f.add_subplot(gs[0, :-1])
#     ax1.plot(posez)
#     ax1.grid()
#     ax1.set_title('position')
#
#     ax2 = f.add_subplot(gs[1, :-1])
#     ax2.plot(posez, "g")
#     ax2.grid()
#     ax2.set_title('velocity')
#
#     ax3 = f.add_subplot(gs[2, :-1])
#     ax3.plot(posez, "r")
#     ax3.grid()
#     ax3.set_title('acceleration')
#
#     plt.show()
#
# showPlots()


class MatModelQuadro:
    def __init__(self):

        pass


class MatrixOfRotation:
    def __init__(self):
        pass



# xoxo.printing()
# print(xoxo.desired_position())
# print(xoxo.desired_z_position(0))


