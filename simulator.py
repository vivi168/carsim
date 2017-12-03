import math
import numpy as np
import csv

class Constant:
  RHO = 1.205
  PI = math.pi
  G = 9.80665002864

class Wheel:
  def __init__(self,
               radius, mass, tire_mu=0, tire_pressure=0, nb=1):
    self.radius = radius # radius of wheel + tire, m
    self.mass = mass # mass of wheel + tire, kg
    self.tire_mu = tire_mu # tire mu
    self.tire_pressure = tire_pressure # tire pressure, bar

    # wheel moment of inertia, kg*m^2
    self.I = (self.mass * self.radius**2 / 2)  * nb

  @staticmethod
  def tire_spec_to_wheel_radius(tire_width, aspect_ratio, rim_diameter):
    # example : P225 / 45R17
    return ((rim_diameter * 2.54 / 2) + (tire_width/10 * aspect_ratio/100)) / 100 # m

class Transmission:
  def __init__(self,
               shift_time, ratios, diff_ratios, friction_loss):
    self.shift_time = shift_time # shift time, s
    self.ratios = ratios # gears ratios
    self.diff_ratios = diff_ratios # differential ratio
    self.friction_loss = friction_loss
    self.current_gear = 1

  def get_ratio(self, for_gear=None):
    if not for_gear:
      for_gear = self.current_gear
    if for_gear < 1:
      for_gear = 1
    if for_gear > len(self.ratios):
      for_gear = len(self.ratios)

    return self.ratios[for_gear - 1] * self.diff_ratios[for_gear - 1]

  def shift_up(self):
    if self.current_gear < len(self.ratios):
      self.current_gear += 1

class Engine:
  def __init__(self,
               rpm_p, torque_p):

    self.rpm_p = rpm_p # list of rpm values, x axis
    self.torque_p = torque_p # list of torque values, y axis
    self.current_rpm = 0

  def get_torque(self, for_rpm=None):
    if not for_rpm:
      for_rpm = self.current_rpm

    # linear interpolation to get torque for a given rpm
    return np.interp(for_rpm, self.rpm_p, self.torque_p)

class Car:
  def __init__(self,
               mass, wheel_base_length, CG_offset, CG_height,
               rear_wheel, front_wheel, flywheel,
               C_d, A,
               transmission, engine, rwd=False):

    self.mass = mass # vehicle mass, kg
    self.wheel_base_length = wheel_base_length # length of wheel base, m
    self.CG_offset = CG_offset # offset of center of gravity, from front, m
    self.CG_height = CG_height # height of center of gravity, m

    self.rear_wheel = rear_wheel
    self.front_wheel = front_wheel
    self.flywheel = flywheel
    self.rwd = rwd

    if self.rwd:
      self.drive_wheel = self.rear_wheel
    else:
      self.drive_wheel = self.front_wheel

    self.C_d = C_d # drag coefficient
    self.A = A # car frontal area

    self.transmission = transmission
    self.engine = engine

    self.v = 0 # velocity, m.s^-1
    self.s = 0 # position, m

  def W_f(self, a=0):
    # Wf = (c/L)*W - (h/L)*M*a, N
    c = self.wheel_base_length/2 - self.CG_offset
    return (c/self.wheel_base_length) * self.mass * Constant.G  - (self.CG_height/self.wheel_base_length) * self.mass * a

  def W_r(self, a=0):
    # Wr = (b/L)*W + (h/L)*M*a, N
    b = self.wheel_base_length/2 + self.CG_offset
    return (b/self.wheel_base_length) * self.mass * Constant.G + (self.CG_height/self.wheel_base_length) * self.mass * a

  def F_drag(self):
    return 0.5 * self.C_d * self.A * Constant.RHO * self.v**2

  def C_rr(self):
    return 0.005 + (1 / self.drive_wheel.tire_pressure) * (0.01 + 0.0095 * (self.v*3.6 / 100)**2)

  def F_rr(self):
    return self.C_rr() * self.mass * Constant.G

  def F_max(self, a=0):
    if self.rwd:
      W = self.W_r(a)
    else:
      W = self.W_f(a)

    return self.drive_wheel.tire_mu * W

def main():
  #radius, mass, tire_mu=0, tire_pressure, nb=1

  wheel_radius = Wheel.tire_spec_to_wheel_radius(225, 45, 17)
  front_wheel = Wheel(wheel_radius, 21, 1.02, 2.05, 2)
  rear_wheel = Wheel(wheel_radius, 21, 1.02, 2, 2)
  flywheel = Wheel(0.114, 9)

  #shift_time, ratios, diff_ratio, friction_loss
  # ratios = [13.2384, 8.2346, 5.7918, 4.334, 3.4299, 2.8737]
  ratios = [3.360, 2.090, 1.470, 1.100, 1.110, 0.930]
  diff_ratios = [3.940, 3.940, 3.940, 3.940, 3.090, 3.090]
  tr = Transmission(0.008, ratios, diff_ratios, 0.15)

  # rpm_p, torque_p
  rp = [1400, 1600, 2500, 3200, 4000, 5000, 5500, 6000, 6250, 6400, 6500]
  tp = [41, 136, 285, 283, 271, 258, 244, 221, 203, 197, 88]
  eg = Engine(rp, tp)

  # mass, wheel_base_length, CG_offset, CG_height,
  # rear_wheel, front_wheel, flywheel,
  # C_d, A,
  # transmission, engine
  car = Car(1336, 2.578, 0, 1,
                 rear_wheel, front_wheel, flywheel,
                 0.32, 2.230,
                 tr, eg)

  s = 0.0 # distance, m
  v = 0.0 # velocity, m/s
  launch_rpm = 3000
  red_line = 6400

  t_step = 0.01 # time step, s
  t_max = 90.0 # end time, s
  step = 0
  t = 0
  a = 0

  t_0_30 = 0
  t_0_60 = 0
  t_0_100 = 0
  t_1_4_m = 0
  v_1_4_m = 0

  result = [[
    "time: ",
    "RPM: ",
    "gear: ",
    "T_eng: ",
    "T_wheel: ",
    "F_wheel: ",
    "F_wheel_max: ",
    "F_drive",
    "F_drag: ",
    "F_rr: ",
    "F_net: ",
    "a: ",
    "v: ",
    "s: "
  ]]

  while t < t_max:
    car.engine.current_rpm = (car.v / car.drive_wheel.radius) * car.transmission.get_ratio() * 60 / (2*Constant.PI)

    if car.transmission.current_gear == 1 and car.engine.current_rpm < launch_rpm:
      car.engine.current_rpm = launch_rpm

    if car.engine.current_rpm > red_line:
      car.transmission.shift_up()
      car.engine.current_rpm = (car.v / car.drive_wheel.radius) * car.transmission.get_ratio() * 60 / (2*Constant.PI)


    T_eng = car.engine.get_torque()
    T_wheel = T_eng * car.transmission.get_ratio() * (1 - car.transmission.friction_loss)
    F_wheel = T_wheel / car.drive_wheel.radius
    F_wheel_max = car.F_max(a) * 2

    if F_wheel > F_wheel_max:
      F_drive = F_wheel_max
    else:
      F_drive = F_wheel

    F_drag = car.F_drag()
    F_rr = car.F_rr()
    F_net = F_drive - F_drag - F_rr

    a = F_net / car.mass
    car.v = a * t_step + car.v
    car.s = 0.5 * a * t_step**2 + car.v * t_step + car.s

    if (car.v * 3.6 / 1.61) > 30 and t_0_30 == 0:
      t_0_30 = t
    if (car.v * 3.6 / 1.61) > 60 and t_0_60 == 0:
      t_0_60 = t
    if (car.v * 3.6 / 1.61) > 100 and t_0_100 == 0:
      t_0_100 = t
    if car.s  > 400 and t_1_4_m == 0 and v_1_4_m == 0:
      t_1_4_m = t
      v_1_4_m = car.v * 3.6 / 1.61


    result.append([
      t,
      car.engine.current_rpm,
      car.transmission.current_gear,
      T_eng,
      T_wheel,
      F_wheel,
      F_wheel_max,
      F_drive,
      F_drag,
      F_rr,
      F_net,
      a,
      car.v,
      car.s
    ])

    step = step + 1
    t = step * t_step

  print("0-30mph: " + str(t_0_30))
  print("0-60mph: " + str(t_0_60))
  print("0-100mph: " + str(t_0_100))
  print("1/4 mi spd: " + str(v_1_4_m))
  print("1/4 mi: " + str(t_1_4_m))
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