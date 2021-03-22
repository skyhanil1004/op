from selfdrive.car.landrover import landrovercan
from selfdrive.can.packer import CANPacker
import binascii

from cereal import car
VisualAlert = car.CarControl.HUDControl.VisualAlert
AudibleAlert = car.CarControl.HUDControl.AudibleAlert

import unittest


class TestLandroverCan(unittest.TestCase):

  """
  def test_checksum(self):
    self.assertEqual(0x75, landrovercan.calc_checksum([0x01, 0x20]))
    self.assertEqual(0xcc, landrovercan.calc_checksum([0x14, 0, 0, 0, 0x20]))

  def test_hud(self):
    packer = CANPacker('chrysler_pacifica_2017_hybrid')
    self.assertEqual(
        [0x2a6, 0, '0100010100000000'.decode('hex'), 0],
        landrovercan.create_lkas_hud(
            packer,
            'park', False, False, 1, 0))
    self.assertEqual(
        [0x2a6, 0, '0100010000000000'.decode('hex'), 0],
        landrovercan.create_lkas_hud(
            packer,
            'park', False, False, 5*4, 0))
    self.assertEqual(
        [0x2a6, 0, '0100010000000000'.decode('hex'), 0],
        landrovercan.create_lkas_hud(
            packer,
            'park', False, False, 99999, 0))
    self.assertEqual(
        [0x2a6, 0, '0200060000000000'.decode('hex'), 0],
        landrovercan.create_lkas_hud(
            packer,
            'drive', True, False, 99999, 0))
    self.assertEqual(
        [0x2a6, 0, '0264060000000000'.decode('hex'), 0],
        landrovercan.create_lkas_hud(
            packer,
            'drive', True, False, 99999, 0x64))
  """

  def test_command(self):
    packer = CANPacker('landrover_rangerover_2017_vogue')
    print(   binascii.hexlify(landrovercan.create_lkas_command(
            packer,
            -4, True, 6)[2]) ) 
    """
    self.assertEqual(
        [0x28f, 0, '77ffff8400700000'.decode('hex'), 0],
        landrovercan.create_lkas_command(
            packer,
            0, True, 0))
    # counter 6, steer -4
    self.assertEqual(
        [0x28f, 0, '5effffb3fc700000'.decode('hex'), 0],
        landrovercan.create_lkas_command(
            packer,
            -4, True, 6))
    """


if __name__ == '__main__':
  unittest.main()
