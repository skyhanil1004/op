import numpy as np
from opendbc.can.parser import CANParser
from opendbc.can.can_define import CANDefine
from selfdrive.config import Conversions as CV
from selfdrive.car.landrover.values import DBC, STEER_THRESHOLD
from common.kalman.simple_kalman import KF1D
from selfdrive.swaglog import cloudlog

def parse_gear_shifter(can_gear):
  if can_gear == 0x0:
    return "park"
  elif can_gear == 0x9:
    return "reverse"
  elif can_gear == 0x12:
    return "neutral"
  elif can_gear == 0x1b:
    return "drive"
  elif can_gear == 0xbb:
    return "sport"
  return "unknown"


def get_can_parser(CP):

  signals = [
    # sig_name, sig_address, default
    ("STEER_RATE00", "EPS_00", 0),
    ("STEER_ANGLE01", "EPS_01", 0),
    ("STEER_SPEED01", "EPS_01", 0),
    ("STEER_TORQUE_DRIVER02", "EPS_02", 0),
    ("STEER_TORQUE_MOTOR02", "EPS_02", 0),
    ("STEER_TORQUE_DRIVER03", "EPS_03", 0),
    ("STEER_TORQUE_EPS04", "EPS_04", 0),
    ("GEAR_SHIFT", "GEAR_PRND", 0),
    ("CRUISE_ON", "CRUISE_CONTROL", 0),
    ("DRIVER_BRAKE", "CRUISE_CONTROL", 0),
    ("SPEED_CRUISE_RESUME", "CRUISE_CONTROL", 1),
    ("ACCELATOR_DRIVER", "ACCELATOR_DRIVER", 0),
    ("SEAT_BELT_DRIVER", "SEAT_BELT", 0),
    ("RIGHT_TURN", "TURN_SIGNAL", 0),
    ("LEFT_TURN", "TURN_SIGNAL", 0),
    ("LEFT_RIGHT_BLINK", "TURN_SIGNAL", 0),
    ("COUNTER", "LKAS_RUN", -1),
    ("SPEED01", "SPEED_01", 0),
    ("SPEED02", "SPEED_02", 0),
    ("LEFT_ALERT_1", "LEFT_ALERT", 0),
    ("RIGHT_ALERT_1", "RIGHT_ALERT", 0),
    ("WHEEL_SPEED_FR", "SPEED_04", 0),
    ("WHEEL_SPEED_FL", "SPEED_04", 0),
    ("WHEEL_SPEED_RR", "SPEED_03", 0),
    ("WHEEL_SPEED_RL", "SPEED_03", 0),
    ("GREEN2WHITE_RIGHT", "LKAS_STATUS", 1),
    ("GREEN2WHITE_LEFT", "LKAS_STATUS", 1),
    ("FRONT_CAR_DISTANCE", "LKAS_HUD_STAT", 0),
    ("LADAR_DISTANCE", "LKAS_HUD_STAT", 0),
    ("HIBEAM", "HEAD_LIGHT", 0),
  ]

  # It's considered invalid if it is not received for 10x the expected period (1/f).
  checks = [
    # sig_address, frequency
    ("EPS_00", 0),
    ("EPS_01", 0),
    ("EPS_02", 0),
    ("EPS_03", 0),
    ("EPS_04",  0),
    ("SPEED_01", 0),
    ("SPEED_02", 0),
    ("SPEED_03", 0),
    ("SPEED_04", 0),
    ("CRUISE_CONTROL", 1),
    ("SEAT_BELT", 0),
    ("LKAS_HUD_STAT", 0),
    ("HEAD_LIGHT", 0),
    ("LKAS_STATUS", 0),
  ]

  return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, 0)

def get_camera_parser(CP):
  signals = [
    # sig_name, sig_address, default
    # TODO read in all the other values
    ("COUNTER", "LKAS_RUN", -1),
  ]
  checks = []

  return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, 2)

class CarState():
  def __init__(self, CP):
    self.CP = CP
    # self.can_define = CANDefine(DBC[CP.carFingerprint]['pt'])
    self.left_blinker_on = 0
    self.right_blinker_on = 0
    self.left_alert = 0
    self.right_alert = 0
    
    self.angle_steers = 0.0

    # initialize can parser
    self.car_fingerprint = CP.carFingerprint

    # vEgo kalman filter
    dt = 0.01
    # Q = np.matrix([[10.0, 0.0], [0.0, 100.0]])
    # R = 1e3
    self.v_ego_kf = KF1D(x0=[[0.0], [0.0]],
                         A=[[1.0, dt], [0.0, 1.0]],
                         C=[1.0, 0.0],
                         K=[[0.12287673], [0.29666309]])
    self.v_ego = 0.0

    self.prev_angle_steers = 0.0
    self.angle_rate_multi = 1.0

  def update(self, cp, cp_cam):

    # update prevs, update must run once per loop
    self.door_all_closed = 1
    self.prev_left_blinker_on = self.left_blinker_on
    self.prev_right_blinker_on = self.right_blinker_on
    
    self.prev_angle_steers = float(self.angle_steers)

    self.seatbelt = (cp.vl["SEAT_BELT"]["SEAT_BELT_DRIVER"]  == 1)

    self.brake_pressed = (cp.vl["CRUISE_CONTROL"]['DRIVER_BRAKE'] == 1) # human-only
    
    self.pedal_gas = 0 # cp.vl["ACCELATOR_DRIVER"]['ACCELATOR_DRIVER']
    self.car_gas = self.pedal_gas
    
    self.esp_disabled = 0 # (cp.vl["TRACTION_BUTTON"]['TRACTION_OFF'] == 1)

    v_wheel = (cp.vl["SPEED_01"]["SPEED01"] + cp.vl["SPEED_02"]["SPEED02"]) / 2.

    self.v_wheel_fr = v_wheel # cp.vl["SPEED_04"]["WHEEL_SPEED_FR"]
    self.v_wheel_fl = v_wheel # cp.vl["SPEED_04"]["WHEEL_SPEED_FL"]
    self.v_wheel_rr = v_wheel # cp.vl["SPEED_03"]["WHEEL_SPEED_RR"]
    self.v_wheel_rl = v_wheel # cp.vl["SPEED_03"]["WHEEL_SPEED_RL"]

    # Kalman filter
    if abs(v_wheel - self.v_ego) > 2.0:  # Prevent large accelerations when car starts at non zero speed
      self.v_ego_kf.x = [[v_wheel], [0.0]]

    self.v_ego_raw = v_wheel
    v_ego_x = self.v_ego_kf.update(v_wheel)
    self.v_ego = float(v_ego_x[0])
    self.a_ego = float(v_ego_x[1])
    self.standstill = not v_wheel > 0.001

    self.angle_steers = float(cp.vl["EPS_01"]["STEER_ANGLE01"])
    self.angle_steers_rate = (cp.vl["EPS_01"]["STEER_SPEED01"])
    
    # HANIL for landrover steers rate   
    angle_steers_diff = float(self.angle_steers - self.prev_angle_steers) 

    if angle_steers_diff < 0.0:
           self.angle_rate_multi = -4
    else:
        if angle_steers_diff > 0.0:
           self.angle_rate_multi = 4

    self.angle_steers_rate *= self.angle_rate_multi

    self.gear_shifter = parse_gear_shifter(cp.vl['GEAR_PRND']['GEAR_SHIFT'])
    self.main_on = (cp.vl["CRUISE_CONTROL"]['CRUISE_ON'] == 1)  # ACC is green.
    self.left_blinker_on = cp.vl["TURN_SIGNAL"]['LEFT_TURN']
    self.right_blinker_on = cp.vl["TURN_SIGNAL"]['RIGHT_TURN']

    self.left_alert = bool(cp.vl["LEFT_ALERT"]['LEFT_ALERT_1'])
    self.right_alert = bool(cp.vl["RIGHT_ALERT"]['RIGHT_ALERT_1'])

    self.steer_torque_driver = cp.vl["EPS_03"]["STEER_TORQUE_DRIVER03"]
    self.steer_torque_motor = cp.vl["EPS_04"]["STEER_TORQUE_EPS04"]
    
    self.steer_override = abs(self.steer_torque_driver) > STEER_THRESHOLD
    # self.steer_override = False # abs(self.steer_torque_driver) > STEER_THRESHOLD
    steer_state = 1 #cp.vl[""]["LKAS_STATE"]
    self.steer_error = steer_state == 4 or (steer_state == 0 and self.v_ego > self.CP.minSteerSpeed)

    self.user_brake = 0
    self.brake_lights = self.brake_pressed
    self.v_cruise_pcm =  round(cp.vl["CRUISE_CONTROL"]['SPEED_CRUISE_RESUME']) 
    self.pcm_acc_status = self.main_on
    self.generic_toggle =  (cp.vl["HEAD_LIGHT"]["HIBEAM"] == 1)
    self.lkas_counter = cp_cam.vl["LKAS_RUN"]['COUNTER']


