import yaml


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
    def __init__(self):
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
        self.numerOfRotors = self.model_config[6]
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


    def printing(self):
        print(self.Ixx)


class MatModelQuadro:
    def __init__(self):
        pass


class MatrixOfRotation:
    def __init__(self):
        pass


x = ControlSystemQuadro()
x.printing()
