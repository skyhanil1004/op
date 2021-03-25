from selfdrive.car import apply_toyota_steer_torque_limits
from selfdrive.car.landrover.landrovercan import create_lkas_command, make_can_msg, create_lkas_hud
#from selfdrive.car.landrover.values import ECU, CAR, STATIC_MSGS, SteerLimitParams
from selfdrive.car.landrover.values import CAR, STATIC_MSGS, CarControllerParams
from opendbc.can.packer import CANPacker
from selfdrive.swaglog import cloudlog


class CarController():
  def __init__(self, dbc_name, CP, VM):
    #self.braking = False
    # redundant safety check with the board
    self.apply_steer_last = 0
    self.ccframe = 0
    self.prev_frame = -1
    self.hud_count = 0
    self.car_fingerprint = CP.carFingerprint
    self.gone_fast_yet = False
    self.steer_rate_limited = False

    self.packer = CANPacker(dbc_name)

  def update(self, enabled, CS, frame, actuators, pcm_cancel_cmd, hud_alert):
    frame = CS.lkas_counter
    if self.prev_frame == frame:
      return[]

    new_steer = int(round(actuators.steer * CarControllerParams.STEER_MAX))
    #apply_std_steer_torque_limits(apply_steer, self.apply_stee    r_last, CS.steer_torque_driver, SteerLimitParams)

    apply_steer = apply_toyota_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorqueEps, CarControllerParams)

    self.steer_rate_limited = new_steer != apply_steer


    moving_fast = CS.out.vEgo > CS.CP.minSteerSpeed  # for status message
    if CS.out.vEgo > (CS.CP.minSteerSpeed - 0.5):  # for command high bit
      self.gone_fast_yet = True
    elif self.car_fingerprint in (CAR.RANGEROVER_2017_VOGUE):
      if CS.out.vEgo < (CS.CP.minSteerSpeed - 3.0):
        self.gone_fast_yet = False  # < 14.5m/s stock turns off this bit, but fine down to 13.5
    lkas_active = moving_fast and enabled

    if not lkas_active:
      apply_steer = 0

    self.apply_steer_last = apply_steer

    can_sends = []

    for (addr, ecu, cars, bus, fr_step, vl) in STATIC_MSGS:
       if(self.ccframe % fr_step == 0):  # 25 0.25s period
           can_sends.append(make_can_msg(addr, vl, bus ))

    if(self.ccframe % 20 == 0):
       can_sends.append(create_lkas_hud(self.packer, left_line, right_line, left_lane_depart, right_lane_depart ))

    if(self.ccframe % 2 == 0):  #  change 0.02s period stock lkas 0.04s
       new_msg = create_lkas_command(self.packer, int(apply_steer), self.gone_fast_yet, frame)
       can_sends.append(new_msg)

    #new_msg = create_lkas_command(self.packer, int(apply_steer), self.gone_fast_yet, frame)
    can_sends.append(new_msg)

    self.ccframe += 1
    self.prev_frame = frame

    return can_sends
