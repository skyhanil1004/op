from cereal import car
from selfdrive.car import apply_toyota_steer_torque_limits
from selfdrive.car.landrover.landrovercan import create_lkas_command, make_can_msg, create_lkas_hud
from selfdrive.car.landrover.values import ECU, CAR, STATIC_MSGS, SteerLimitParams
from opendbc.can.packer import CANPacker
from selfdrive.swaglog import cloudlog


AudibleAlert = car.CarControl.HUDControl.AudibleAlert
LOUD_ALERTS = [AudibleAlert.chimeWarning1, AudibleAlert.chimeWarning2, AudibleAlert.chimeWarningRepeat]

class CarController():
  def __init__(self, dbc_name, car_fingerprint, enable_camera):
    self.braking = False
    # redundant safety check with the board
    self.controls_allowed = True
    self.apply_steer_last = 0
    self.ccframe = 0
    self.lkascnt = 0
    self.prev_frame = -1
    self.hud_count = 0
    self.car_fingerprint = car_fingerprint
    self.alert_active = False
    self.send_chime = False
    self.gone_fast_yet = False

    self.fake_ecus = set()
    if enable_camera:
      self.fake_ecus.add(ECU.CAM)

    self.packer = CANPacker(dbc_name)

  def update(self, enabled, CS, frame, actuators, pcm_cancel_cmd, hud_alert, 
             left_line, right_line, left_lane_depart, right_lane_depart):

    """
    # init safety test lines
    if CS.generic_toggle:
        actuators.steer = 1
    apply_steer = int(round(actuators.steer * SteerLimitParams.STEER_MAX))
    # cloudlog.warning("LANDROVER LKAS aplly_steer(%d)", actuators.steer)
    # end safety test line
    """

    # *** compute control surfaces ***
    # steer torque

    apply_steer = actuators.steer * SteerLimitParams.STEER_MAX
    #apply_std_steer_torque_limits(apply_steer, self.apply_stee    r_last, CS.steer_torque_driver, SteerLimitParams)
    apply_steer = apply_toyota_steer_torque_limits(apply_steer, self.apply_steer_last, CS.steer_torque_motor, SteerLimitParams)

    # """
    moving_fast = CS.v_ego > CS.CP.minSteerSpeed  # for status message
    if CS.v_ego > (CS.CP.minSteerSpeed - 0.5):  # for command high bit
      self.gone_fast_yet = True
    elif self.car_fingerprint in (CAR.RANGEROVER_2017_VOGUE):
      if CS.v_ego < (CS.CP.minSteerSpeed - 3.0):
        self.gone_fast_yet = False  # < 14.5m/s stock turns off this bit, but fine down to 13.5
    lkas_active = moving_fast and enabled
    # cloudlog.warning("LANDROVER LKAS aplly_steer(%d)%d", apply_steer, actuators.steer)
    if not lkas_active:
      apply_steer = 0
    # """
    
    # cloudlog.warning("LKAS aplly_steer[%d] - %d", apply_steer, actuators.steer)
    
    if (not enabled or CS.left_blinker_on or CS.right_blinker_on) and (CS.v_ego < 16.7192):
      apply_steer = 0

    self.apply_steer_last = apply_steer

    """
    if audible_alert in LOUD_ALERTS:
      self.send_chime = True
    """
    
    can_sends = []

    #*** control msgs ***

    """
    if self.send_chime:
      new_msg = create_chimes(AudibleAlert)
      can_sends.append(new_msg)
      if audible_alert not in LOUD_ALERTS:
        self.send_chime = False
    """

    """
    if pcm_cancel_cmd:
      # TODO: would be better to start from frame_2b3
      new_msg = create_wheel_buttons(self.ccframe)
      can_sends.append(new_msg)
    """

    # LKAS_HEARTBIT is forwarded by Panda so no need to send it here.
    # frame is 100Hz (0.01s period)

    """
    if (self.ccframe % 25 == 0):  # 0.25s period
      if (CS.lkas_car_model != -1):
        new_msg = create_lkas_hud(
            self.packer, CS.gear_shifter, lkas_active, hud_alert,
            self.hud_count, CS.lkas_car_model)
        can_sends.append(new_msg)
        self.hud_count += 1
    """

    for (addr, ecu, cars, bus, fr_step, vl) in STATIC_MSGS:
       if(self.ccframe % fr_step == 0):  # 25 0.25s period
           can_sends.append(make_can_msg(addr, vl, bus ))

    if(self.ccframe % 20 == 0):
       can_sends.append(create_lkas_hud(self.packer, left_line, right_line, left_lane_depart, right_lane_depart ))


    if(self.ccframe % 2 == 0):  #  change 0.02s period stock lkas 0.04s
       new_msg = create_lkas_command(self.packer, int(apply_steer), self.gone_fast_yet, self.lkascnt)
       self.lkascnt += 1
       #cloudlog.warning("LANDROVER LKAS (%s)", new_msg)
       can_sends.append(new_msg)

    self.ccframe += 1
    self.prev_frame = frame

    return can_sends
