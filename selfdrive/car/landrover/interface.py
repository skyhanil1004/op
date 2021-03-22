#!/usr/bin/env python3
from common.realtime import sec_since_boot
from cereal import car
from selfdrive.config import Conversions as CV
from selfdrive.controls.lib.drive_helpers import EventTypes as ET, create_event
from selfdrive.controls.lib.vehicle_model import VehicleModel
from selfdrive.car.landrover.carstate import CarState, get_can_parser, get_camera_parser
# from selfdrive.car.landrover.values import ECU, check_ecu_msgs, CAR
from selfdrive.car.landrover.values import ECU, ECU_FINGERPRINT, CAR, FINGERPRINTS
# from selfdrive.car import STD_CARGO_KG, scale_rot_inertia, scale_tire_stiffness
from selfdrive.car import STD_CARGO_KG, scale_rot_inertia, scale_tire_stiffness, is_ecu_disconnected, gen_empty_fingerprint
from selfdrive.swaglog import cloudlog


class CarInterface():
  def __init__(self, CP, CarController):
    self.CP = CP
    self.VM = VehicleModel(CP)

    self.frame = 0
    self.gas_pressed_prev = False
    self.brake_pressed_prev = False
    self.cruise_enabled_prev = False
    self.low_speed_alert = False

    # *** init the major players ***
    self.CS = CarState(CP)
    self.cp = get_can_parser(CP)
    self.cp_cam = get_camera_parser(CP)

    self.CC = None
    if CarController is not None:
      self.CC = CarController(self.cp.dbc_name, CP.carFingerprint, CP.enableCamera)

  @staticmethod
  def compute_gb(accel, speed):
    return float(accel) / 3.0

  @staticmethod
  def calc_accel_override(a_ego, a_target, v_ego, v_target):
    return 1.0

  @staticmethod
  def get_params(candidate, fingerprint, vin="", has_relay=True):  # is_panda_black=False):

    ret = car.CarParams.new_message()

    ret.carName = "landrover"
    ret.carFingerprint = candidate
    ret.carVin = "SALGA2FE0HA367795" # vin
    ret.isPandaBlack = True    # is_panda_black
    ret.radarOffCan = True

    ret.safetyModel = car.CarParams.SafetyModel.landrover

    # pedal
    ret.enableCruise = True

    # Speed conversion:              20, 45 mph
    ret.wheelbase = 2.922  # in meters for rangerover vogue  2017
    # ret.steerRatio = 15.2 # Pacifica Hybrid 2017 is 16.2
    ret.steerRatio = 11.2 # Pacifica Hybrid 2017 is 16.2
    ret.mass = 2500. + STD_CARGO_KG  # kg curb weight Pacifica Hybrid 2017
    tire_stiffness_factor = 0.5371   # hand-tun

    ret.lateralTuning.pid.kpBP, ret.lateralTuning.pid.kiBP = [[9., 20.], [9., 20.]]
    # ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[0.], [0.]]
    # ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.05,0.0.6], [0.001,0.002]]
    ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.01,0.02], [0.0005,0.001]]
    #ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.05], [0.001]]
    ret.lateralTuning.pid.kf = 0.00001   # full torque for 10 deg at 80mph means 0.00007818594
    ret.steerActuatorDelay = 0.025       # 0.05 -> 0.065 12/11 modify org 0.1 -> 0.05 then good straight
    ret.steerRateCost = 0.5              # org 0.7

    ret.centerToFront = ret.wheelbase * 0.44 
    ret.minSteerSpeed = 1.8    # m/s
    ret.minEnableSpeed = -1.   # enable is done by stock ACC, so ignore this

    # TODO: get actual value, for now starting with reasonable value for
    # civic and scaling by mass and wheelbase
    ret.rotationalInertia = scale_rot_inertia(ret.mass, ret.wheelbase)

    # TODO: start from empirically derived lateral slip stiffness for the civic and scale by
    # mass and CG position, so all cars will have approximately similar dyn behaviors
    ret.tireStiffnessFront, ret.tireStiffnessRear = scale_tire_stiffness(ret.mass, ret.wheelbase, ret.centerToFront)

    # no rear steering, at least on the listed cars above
    ret.steerRatioRear = 0.

    # steer, gas, brake limitations VS speed
    ret.steerMaxBP = [16. * CV.KPH_TO_MS, 45. * CV.KPH_TO_MS]  # breakpoints at 1 and 40 kph
    ret.steerMaxV = [1., 1.]  # 2/3rd torque allowed above 45 kph
    ret.gasMaxBP = [0.]
    ret.gasMaxV = [0.5]
    ret.brakeMaxBP = [5., 20.]
    ret.brakeMaxV = [1., 0.8]

    ret.enableCamera = True  # not check_ecu_msgs(fingerprint, ECU.CAM) or is_panda_black
    print("ECU Camera Simulated: {0}".format(ret.enableCamera))
    ret.openpilotLongitudinalControl = False

    ret.steerLimitAlert = True
    ret.stoppingControl = False
    ret.startAccel = 0.0

    ret.longitudinalTuning.deadzoneBP = [0., 9.]
    ret.longitudinalTuning.deadzoneV = [0., .15]
    ret.longitudinalTuning.kpBP = [0., 5., 35.]
    ret.longitudinalTuning.kpV = [3.6, 2.4, 1.5]
    ret.longitudinalTuning.kiBP = [0., 35.]
    ret.longitudinalTuning.kiV = [0.54, 0.36]


    return ret

  # returns a car.CarState
  def update(self, c, can_strings):
    # ******************* do can recv *******************
    # self.cp.update_strings(int(sec_since_boot() * 1e9), can_strings)
    # self.cp_cam.update_strings(int(sec_since_boot() * 1e9), can_strings)
    self.cp.update_strings(can_strings)
    self.cp_cam.update_strings(can_strings)

    self.CS.update(self.cp, self.cp_cam)

    # create message
    ret = car.CarState.new_message()

    ret.canValid = self.cp.can_valid and self.cp_cam.can_valid

    # speeds
    ret.vEgo = self.CS.v_ego
    ret.vEgoRaw = self.CS.v_ego_raw
    ret.aEgo = self.CS.a_ego
    ret.yawRate = self.VM.yaw_rate(self.CS.angle_steers * CV.DEG_TO_RAD, self.CS.v_ego)
    ret.standstill = self.CS.standstill
    ret.wheelSpeeds.fl = self.CS.v_wheel_fl
    ret.wheelSpeeds.fr = self.CS.v_wheel_fr
    ret.wheelSpeeds.rl = self.CS.v_wheel_rl
    ret.wheelSpeeds.rr = self.CS.v_wheel_rr

    # gear shifter
    ret.gearShifter = self.CS.gear_shifter

    # gas pedal
    ret.gas = self.CS.car_gas
    ret.gasPressed = self.CS.pedal_gas > 0

    # brake pedal
    ret.brake = self.CS.user_brake
    ret.brakePressed = self.CS.brake_pressed
    ret.brakeLights = self.CS.brake_lights

    # steering wheel
    ret.steeringAngle = self.CS.angle_steers
    ret.steeringRate = self.CS.angle_steers_rate

    ret.steeringTorque = self.CS.steer_torque_driver
    ret.steeringPressed = self.CS.steer_override
    ret.steeringTorqueEps = self.CS.steer_torque_motor

    # cruise state
    ret.cruiseState.enabled = self.CS.pcm_acc_status  # same as main_on
    ret.cruiseState.speed = self.CS.v_cruise_pcm * CV.KPH_TO_MS
    ret.cruiseState.available = self.CS.main_on
    ret.cruiseState.speedOffset = 0.
    # ignore standstill in hybrid rav4, since pcm allows to restart without
    # receiving any special command
    ret.cruiseState.standstill = False

    # TODO: button presses
    buttonEvents = []

    # gernby kegman
    #ret.readdistancelines = self.CS.read_distance_lines

    if self.CS.left_blinker_on != self.CS.prev_left_blinker_on:
      be = car.CarState.ButtonEvent.new_message()
      be.type = 'leftBlinker'
      # be.type = ButtonType.leftBlinker
      be.pressed = self.CS.left_blinker_on != 0
      buttonEvents.append(be)

    if self.CS.right_blinker_on != self.CS.prev_right_blinker_on:
      be = car.CarState.ButtonEvent.new_message()
      be.type = 'rightBlinker'
      # be.type = ButtonType.rightBlinker
      be.pressed = self.CS.right_blinker_on != 0
      buttonEvents.append(be)

    ret.buttonEvents = buttonEvents
    ret.leftBlinker = bool(self.CS.left_blinker_on)
    ret.rightBlinker = bool(self.CS.right_blinker_on)

    ret.leftAlert = bool(self.CS.left_alert)
    ret.rightAlert = bool(self.CS.right_alert)

    ret.doorOpen = not self.CS.door_all_closed
    ret.seatbeltUnlatched = not self.CS.seatbelt
    self.low_speed_alert = (ret.vEgo < self.CP.minSteerSpeed)

    ret.genericToggle = self.CS.generic_toggle
    #ret.lkasCounter = self.CS.lkas_counter
    #ret.lkasCarModel = self.CS.lkas_car_model

    # events
    events = []
    if not (ret.gearShifter in ('drive', 'low')):
      events.append(create_event('wrongGear', [ET.NO_ENTRY, ET.SOFT_DISABLE]))
    if ret.doorOpen:
      events.append(create_event('doorOpen', [ET.NO_ENTRY, ET.SOFT_DISABLE]))
    if ret.seatbeltUnlatched:
      events.append(create_event('seatbeltNotLatched', [ET.NO_ENTRY, ET.SOFT_DISABLE]))
    if self.CS.esp_disabled:
      events.append(create_event('espDisabled', [ET.NO_ENTRY, ET.SOFT_DISABLE]))
    if not self.CS.main_on:
      events.append(create_event('wrongCarMode', [ET.NO_ENTRY, ET.USER_DISABLE]))
    if ret.gearShifter == 'reverse':
      events.append(create_event('reverseGear', [ET.NO_ENTRY, ET.IMMEDIATE_DISABLE]))
    if self.CS.steer_error:
      events.append(create_event('steerUnavailable', [ET.NO_ENTRY, ET.IMMEDIATE_DISABLE, ET.PERMANENT]))

    if ret.cruiseState.enabled and not self.cruise_enabled_prev:
      events.append(create_event('pcmEnable', [ET.ENABLE]))
    elif not ret.cruiseState.enabled:
      events.append(create_event('pcmDisable', [ET.USER_DISABLE]))

    # disable on gas pedal and speed isn't zero. Gas pedal is used to resume ACC
    # from a 3+ second stop.
    if (ret.gasPressed and (not self.gas_pressed_prev) and ret.vEgo > 2.0):
      events.append(create_event('pedalPressed', [ET.NO_ENTRY, ET.USER_DISABLE]))

    if self.low_speed_alert:
      events.append(create_event('belowSteerSpeed', [ET.WARNING]))

    ret.events = events

    self.gas_pressed_prev = ret.gasPressed
    self.brake_pressed_prev = ret.brakePressed
    self.cruise_enabled_prev = ret.cruiseState.enabled

    return ret.as_reader()

  # pass in a car.CarControl
  # to be called @ 100hz
  def apply(self, c):
    can_sends = self.CC.update(c.enabled, self.CS, self.frame,
                               c.actuators, c.cruiseControl.cancel, c.hudControl.visualAlert,
                               c.hudControl.leftLaneVisible, c.hudControl.rightLaneVisible,  
                               c.hudControl.leftLaneDepart, c.hudControl.rightLaneDepart
                               )
    self.frame += 1

    return can_sends
