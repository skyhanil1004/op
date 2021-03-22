from cereal import car
from selfdrive.car.landrover.lkas_tbl import find_steer_torq
from selfdrive.car import make_can_msg
import binascii
import codecs

# 15 all green
# 1d left green, right white
# 35 left white, right green,
def create_lkas_hud(packer, left_line, right_line, left_lane_depart, right_lane_depart):
  values = {
    "GREEN2WHITE_RIGHT": 2 if right_lane_depart else 1 if right_line else 3,
    "GREEN2WHITE_LEFT": 2 if left_lane_depart else 1 if left_line else 3,
    "NEW_41" : 0x41,
    "NEW_01" : 1,
    "NEW_0d" : 0xd,
    "NEW_1_1": 1,
    "NEW_e7" : 0xe7,
    "NEW_2" : 2,
    "NEW_ed" : 0xed,
    "NEW_00" : 0
  }

  return packer.make_can_msg("LKAS_STATUS", 0, values)

def create_lkas_command(packer, apply_steer, moving_fast, frame):
  # LKAS_COMMAND 0x28F (655) Lane-keeping signal to turn the wheel.
  counter = frame % 0x10
  values = {
    "ALLFFFF" : 0xffff,
    "A1" : 1,
    "HIGH_TORQ": 0,
    "ALL11" : 3,
    "COUNTER" : counter,
    "STEER_TORQ": apply_steer,
    "LKAS_GREEN" : 1
  }

  #       cs                cnt3  torq 
  #       0      1     2     3     4
  dat = [ 0xeb, 0xff, 0xff, 0xe4, 0x00, 0x70, 0x00, 0x00 ]

  torq , dat[0] = find_steer_torq(counter, apply_steer)
  # torq , dat[0] = find_steer_torq(counter, 0)

  dat[3] = (((counter << 3) | ((torq & 0x700) >> 8)) | 0x80)
  # dat[3] = ((counter << 3) | ((torq & 0x700) >> 8)) 
  dat[4] = torq & 0xFF

  candat = binascii.hexlify(bytearray(dat))
  #cloudlog.warning("LANDROVER LKAS %d %x (%x:%x) %s", apply_steer, apply_steer, counter, torq, candat)
  return  make_can_msg(0x28F, codecs.decode(candat, 'hex'), 0)
  
