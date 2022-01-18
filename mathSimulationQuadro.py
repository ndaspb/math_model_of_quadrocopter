import yaml


def read_yaml(t):
    """
    Read parameters of quad-rotor in Yaml file

    param t: string
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
        self.model_config = read_yaml('quadModelConfig.yaml')
        self.control_system = read_yaml('quadControlSystem.yaml')
        print(self.model_config)
        print(self.control_system)
        self.mass = self.model_config[2]
        self.lengthOfFlyerArms = self.model_config[5]
        self.motorThrustCoef = self.model_config[3]
        self.motorResistCoef = self.model_config[4]
        self.numerOfRotors = self.model_config[6]
        self.Ixx = self.model_config[7]
        self.Iyy = self.model_config[8]
        self.Izz = self.model_config[9]
        self.VelocityRotors = self.model_config[10]
        self.dt = self.model_config[0]
        self.simulationTotalTime = self.model_config[1]

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
