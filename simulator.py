import math
import numpy as np
import csv
import json
import inspect

class Constant:
  RHO = 1.205
  PI = math.pi
  G = 9.80665002864

class ActsAsDict:
    def __init__(self):
        pass
    def init_args(self):
        return inspect.getargspec(self.__init__).args[1:]
    def keys(self):
        return self.init_args()
    def __getitem__(self, key):
        return getattr(self, key)

class Wheel(ActsAsDict):
  def __init__(self, section_width, aspect_ratio, rim_diameter, mu, pressure):
    self.radius = ((rim_diameter * 2.54 / 2) + (section_width/10 * aspect_ratio/100)) / 100 # m
    self.tire_mu = mu # tire friction coefficient
    self.tire_pressure = pressure # tire pressure, bar

class Transmission(ActsAsDict):
  def __init__(self, ratios, diff_ratios, friction_loss):
    self.ratios = ratios # gears ratios
    self.diff_ratios = diff_ratios # differential ratio
    self.friction_loss = friction_loss # gearbox friction loss
    self.current_gear = 1

  def get_ratio(self, for_gear=None):
    if not for_gear:
      for_gear = self.current_gear
    if for_gear < 1:
      for_gear = 1
    if for_gear > len(self.ratios):
      for_gear = len(self.ratios)

    return self.ratios[for_gear - 1] * self.diff_ratios[for_gear - 1]

  def get_net_ratio(self, for_gear=None):
    return self.get_ratio(for_gear) * (1 - self.friction_loss)

  def shift_up(self):
    if self.current_gear < len(self.ratios):
      self.current_gear += 1

class Engine(ActsAsDict):
  def __init__(self, rpm_p, torque_p):

    self.rpm_p = rpm_p # list of rpm values, x axis
    self.torque_p = torque_p # list of torque values, y axis
    self.current_rpm = 0

  def get_torque(self, for_rpm=None):
    if not for_rpm:
      for_rpm = self.current_rpm

    # linear interpolation to get torque for a given rpm
    return np.interp(for_rpm, self.rpm_p, self.torque_p)

class Car:
  def __init__(self, mass, C_d, A, launch_rpm, redline_rpm, engine, transmission, wheel):

    self.mass = mass # vehicle mass, kg
    self.C_d = C_d # drag coefficient
    self.A = A # car frontal area

    self.launch_rpm = launch_rpm
    self.redline_rpm = redline_rpm

    self.wheel = Wheel(**wheel)
    self.transmission = Transmission(**transmission)
    self.engine = Engine(**engine)

    self.a = 0 # acceleration, m.s^-2
    self.v = 0 # velocity, m.s^-1
    self.s = 0 # position, m

  def F_drag(self):
    return 0.5 * self.C_d * self.A * Constant.RHO * self.v**2

  def C_rr(self):
    return 0.005 + (1 / self.wheel.tire_pressure) * (0.01 + 0.0095 * (self.v*3.6 / 100)**2)

  def F_rr(self):
    return self.C_rr() * self.mass * Constant.G

  def F_max(self):
    W = self.mass * Constant.G * 0.5
    return self.wheel.tire_mu * W

  @classmethod
  def from_json(cls, filename):
    with open(filename) as car_data:
      data = json.load(car_data)
      return Car(**data)


def main():

  car = Car.from_json('350z_2005.txt')
  # car = Car.from_json('vw_golf_gti_2007.json')

  t_step = 0.01 # time step, s
  t_max = 360.0 # end time, s
  step = 0
  t = 0

  t_0_30 = 0
  t_0_60 = 0
  t_0_100 = 0
  t_0_120 = 0
  t_1_4_m = 0
  v_1_4_m = 0

  result = [[ "time: ", "RPM: ", "gear: ", "T_eng: ", "T_wheel: ", "F_wheel: ", "F_wheel_max: ", "F_drive", "F_drag: ", "F_rr: ", "F_net: ", "a: ", "v: ", "s: " ]]

  while t < t_max:
    car.engine.current_rpm = (car.v / car.wheel.radius) * car.transmission.get_ratio() * 60 / (2*Constant.PI)

    if car.transmission.current_gear == 1 and car.engine.current_rpm < car.launch_rpm:
      car.engine.current_rpm = car.launch_rpm

    if car.engine.current_rpm > car.redline_rpm:
      car.transmission.shift_up()
      car.engine.current_rpm = (car.v / car.wheel.radius) * car.transmission.get_ratio() * 60 / (2*Constant.PI)

    T_eng = car.engine.get_torque()
    T_wheel = T_eng * car.transmission.get_net_ratio()
    F_wheel = T_wheel / car.wheel.radius
    F_wheel_max = car.F_max()

    if F_wheel > F_wheel_max:
      F_drive = F_wheel_max
    else:
      F_drive = F_wheel

    F_drag = car.F_drag()
    F_rr = car.F_rr()
    F_net = F_drive - F_drag - F_rr

    car.a = F_net / car.mass
    car.v = car.a * t_step + car.v
    car.s = car.v * t_step + car.s

    if (car.v * 3.6 / 1.61) > 30 and t_0_30 == 0:
      t_0_30 = t
    if (car.v * 3.6 / 1.61) > 60 and t_0_60 == 0:
      t_0_60 = t
    if (car.v * 3.6 / 1.61) > 100 and t_0_100 == 0:
      t_0_100 = t
    if (car.v * 3.6 / 1.61) > 120 and t_0_120 == 0:
      t_0_120 = t
    if car.s  > 400 and t_1_4_m == 0 and v_1_4_m == 0:
      t_1_4_m = t
      v_1_4_m = car.v * 3.6 / 1.61


    result.append([ t, car.engine.current_rpm, car.transmission.current_gear, T_eng, T_wheel, F_wheel, F_wheel_max, F_drive, F_drag, F_rr, F_net, car.a, car.v, car.s ])

    step = step + 1
    t = step * t_step

  print("0-30mph: " + str(t_0_30) + "s")
  print("0-60mph: " + str(t_0_60) + "s")
  print("1/4 mi spd: " + str(v_1_4_m) + " mph")
  print("1/4 mi: " + str(t_1_4_m) + "s")
  print("0-100mph: " + str(t_0_100) + "s")
  print("0-120mph: " + str(t_0_120) + "s")
  print(car.transmission.current_gear)
  print(car.engine.current_rpm)
  print(car.v)
  print(car.s)

  # print(result)
  with open("output.csv", "w") as f:
    writer = csv.writer(f)
    writer.writerows(result)


if __name__ == "__main__":
  main()
