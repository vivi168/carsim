import math
import numpy as np

class Constant:
  RHO = 1.205
  PI = math.pi
  G = 9.80665002864

class Wheel:
  def __init__(self,
               radius, mass, tires_mu=0, nb=1):
    self.radius = radius # radius of wheel + tire, m
    self.mass = mass # mass of wheel + tire, kg
    self.tires_mu = tires_mu # tire mu

    # wheel moment of inertia, kg*m^2
    self.I = (self.mass * self.radius**2 / 2)  * nb

  @staticmethod
  def tire_spec_to_wheel_radius(tire_width, aspect_ratio, rim_diameter):
    # example : P225 / 45R17
    return ((rim_diameter * 2.54 / 2) + (tire_width/10 * aspect_ratio/100)) / 100 # m

class Transmission:
  def __init__(self,
               shift_time, ratios, diff_ratio, friction_loss):
    self.shift_time = shift_time
    self.ratios = ratios # gears ratios
    self.diff_ratio = diff_ratio # differential ratio
    self.friction_loss = friction_loss
    self.current_gear = 1

  def getRatio(self, for_gear=None):
    if not for_gear:
      for_gear = self.current_gear
    if for_gear > len(self.ratios):
      for_gear = len(self.ratios)

    return self.ratios[for_gear - 1]

  def getTorque(T_eng):
    return T_eng * self.getRatio() * self.diff_ratio * (1 - self.friction_loss)

class Engine:
  def __init__(self,
               rpm_p, torque_p):

    self.current_rpm = 0
    self.rpm_p = rpm_p
    self.torque_p = torque_p


  def getTorque(self, for_rpm=None):
    if not for_rpm:
      for_rpm = self.current_rpm

    # linear interpolation to get torque for a given rpm
    return np.interp(for_rpm, self.rpm_p, self.torque_p)


    def getPower(self, rpm):
      return self.getTorque(rpm) * self.current_rpm /60 * 2 * Constant.PI;



class Car:
  def __init__(self,
               mass, wheel_base_length, CG_offset, CG_height,
               rear_wheel, front_wheel, flywheel,
               C_d, A,
               transmission, engine):

    self.mass = mass # vehicle mass, kg
    self.wheel_base_length = wheel_base_length # length of wheel base, m
    self.CG_offset = CG_offset # offset of center of gravity, from front, m
    self.CG_height = CG_height # height of center of gravity, m

    self.rear_wheel = rear_wheel
    self.front_wheel = front_wheel
    self.flywheel = flywheel

    self.C_d = C_d # drag coefficient
    self.A = A # car frontal area

    self.transmission = transmission
    self.engine = engine



  def weight_front_wheel(self, accel):
    # Wf = (c/L)*W - (h/L)*M*a
    c = self.wheel_base_length/2 - self.CG_offset
    return (c/self.wheel_base_length) * self.mass * Constant.G  - (self.CG_height/self.wheel_base_length) * self.mass * accel

  def weight_rear_wheel(self, accel):
    # Wr = (b/L)*W + (h/L)*M*a,
    b = self.wheel_base_length/2 + self.CG_offset
    return (b/self.wheel_base_length) * self.mass * Constant.G + (self.CG_height/self.wheel_base_length) * self.mass * accel




rear_wheel = Wheel(0.3, 20)
front_wheel  = Wheel(0,3, 20)
flywheel = Wheel(0.2, 7)

rp = [1400, 1600, 2500, 3200, 4000, 5000, 5500, 6000, 6250, 6400, 6500]
tp = [30, 100, 210, 209, 200, 190, 180, 163, 150, 145, 65]

vr38dett = Engine(rp, tp)

print(vr38dett.getTorque(6456))

tr = Transmission(0, [5,4,3,2,1], 2.75, 0.3)

# print(tr.getRatio(8))

gtr = Car(1500, 2.5, 0, 1, 5, front_wheel, rear_wheel, 0.3, 2.25, tr, vr38dett)

print(gtr.weight_front_wheel(4.9))
print(gtr.weight_rear_wheel(4.9))

# print(Wheel.tire_spec_to_wheel_radius(225, 45, 17))
