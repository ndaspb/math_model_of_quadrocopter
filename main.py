import costants as cs
from controller import QuadcopterController
from model import QuadcopterModel
from simulator import Simulator
# import numpy as np

controller = QuadcopterController()

controller.position_controller_x.set_pid_gains(1, 0, 0)
controller.position_controller_x.set_saturation_limit(-3, 3)
controller.position_controller_x.set_integral_limit(0)

controller.position_controller_y.set_pid_gains(0.3, 0, 0)
controller.position_controller_y.set_saturation_limit(-3, 3)
controller.position_controller_y.set_integral_limit(0)

controller.position_controller_z.set_pid_gains(5, 0, 0)
controller.position_controller_z.set_saturation_limit(-3, 3)
controller.position_controller_z.set_integral_limit(0)

controller.velocity_controller_x.set_pid_gains(0.01, 0, 1)
controller.velocity_controller_x.set_saturation_limit(-0.5, 0.5)
controller.velocity_controller_x.set_integral_limit(1)

controller.velocity_controller_y.set_pid_gains(0.01, 0, 1)
controller.velocity_controller_y.set_saturation_limit(-0.5, 0.5)
controller.velocity_controller_y.set_integral_limit(1)

controller.velocity_controller_z.set_pid_gains(3, 0, 3)
controller.velocity_controller_z.set_saturation_limit(-10, 10)
controller.velocity_controller_z.set_integral_limit(1)

controller.pitch_controller.set_pid_gains(4, 0, 0)
controller.pitch_controller.set_saturation_limit(-5, 5)
controller.pitch_controller.set_integral_limit(0)

controller.roll_controller.set_pid_gains(4, 0, 0)
controller.roll_controller.set_saturation_limit(-5, 5)
controller.roll_controller.set_integral_limit(0)

controller.yaw_controller.set_pid_gains(1, 0, 0)
controller.yaw_controller.set_saturation_limit(-1, 1)
controller.yaw_controller.set_integral_limit(0)

controller.pitch_rate_controller.set_pid_gains(2, 0.1, 1)
controller.pitch_rate_controller.set_saturation_limit(-7, 7)
controller.pitch_rate_controller.set_integral_limit(1)

controller.roll_rate_controller.set_pid_gains(2, 0.1, 0)
controller.roll_rate_controller.set_saturation_limit(-7, 7)
controller.roll_rate_controller.set_integral_limit(1)

controller.yaw_rate_controller.set_pid_gains(1, 0, 0)
controller.yaw_rate_controller.set_saturation_limit(-0.5, 0.5)
controller.yaw_rate_controller.set_integral_limit(0)

target_points = [[0, 0, 2], [0, 0, 5], [0, 0, 10]]

model = QuadcopterModel(cs.quadcopter_inertia, cs.quadcopter_mass, cs.trust_coeff, cs.drag_coeff, cs.arm_length)

sim = Simulator(controller, model, 0.05, target_points)

sim.run()
