import pytest
import re
import unittest

from types import SimpleNamespace

from opendbc.can import CANParser
from opendbc.car import Bus, DT_CTRL, gen_empty_fingerprint, structs
from opendbc.car.structs import CarParams
from opendbc.car.tesla.interface import CarInterface
from opendbc.car.tesla.fingerprints import FW_VERSIONS
from opendbc.car.tesla.radar_interface import RADAR_START_ADDR
from opendbc.car.tesla.carcontroller import CarController
from opendbc.car.tesla.values import CAR, CANBUS, DBC, CarControllerParams

Ecu = CarParams.Ecu

# Fields prefixed unknown_* we observe structurally but don't know the meaning of.
# Only `platform` has evidence-backed semantic meaning (matches car_model in FW_VERSIONS).
#
# unknown_prefix is everything before the comma; we don't split it because we don't know what its
# parts mean, but observed shape is: <family>_<package>_<triplet> (<build>), e.g.
#   TeMYG4 _ Main     _ 0.0.0 (78)     or     TeM3 _ SP_XP002p2 _ 0.0.0 (23)
#   family   package    triplet build           family  package    triplet build
#
# After the comma, the version string decomposes into:
#   platform             : E/Y/X = car model (Model 3 / Y / X). The only field with known meaning.
#   variant_code         : differentiator WITHIN a platform — hardware/trim/calibration bits packed
#                          into <digit?><letters?><3-digit series>, e.g. '4HP015', '4003', 'L014',
#                          'PR003'. We don't fully know what the parts mean individually, but the
#                          whole string identifies a specific variant within the car model.
#   software_major/minor : numeric components after the first '.' — conventional release numbers.
#                          minor is optional (e.g. 'E4S014.27' has no minor).
#
# Suspected (not confirmed): for M3/MY, `TeM3_*` outer + no-leading-digit variant_code == HW3, and
# `TeMYG4_*` outer + leading-'4' variant_code == HW4 (the 'G4' in TeMYG4 likely denotes Gen 4).
#
# Example full parse of 'TeMYG4_Main_0.0.0 (78),E4HP015.05.0':
#   unknown_prefix='TeMYG4_Main_0.0.0 (78)'
#   platform=E  variant_code=4HP015  software_major=05  software_minor=0
FW_RE = re.compile(
  rb'^(?P<unknown_prefix>.+),' +
  rb'(?P<platform>[EYX])' +
  rb'(?P<variant_code>\d?[A-Z]*\d{3})' +
  rb'\.(?P<software_major>\d+)' +
  rb'(?:\.(?P<software_minor>\d+))?$'
)

PLATFORM_TO_CAR = {
  b'E': CAR.TESLA_MODEL_3,
  b'Y': CAR.TESLA_MODEL_Y,
  b'X': CAR.TESLA_MODEL_X,
}


class TestTeslaFingerprint(unittest.TestCase):
  def test_fw_platform_code(self):
    # Every EPS FW must parse and its platform letter must match the car it's filed under.
    for car_model, ecus in FW_VERSIONS.items():
      for fw in ecus.get((Ecu.eps, 0x730, None), []):
        m = FW_RE.match(fw)

        assert m is not None, f"Unparsable FW: {fw}"
        assert PLATFORM_TO_CAR[m['platform']] == car_model, f"Platform letter {m['platform']!r} != {car_model.value}: {fw}"

  def test_radar_detection(self):
    # Test radar availability detection for cars with radar DBC defined
    for radar in (True, False):
      fingerprint = gen_empty_fingerprint()
      if radar:
        fingerprint[1][RADAR_START_ADDR] = 8
      CP = CarInterface.get_params(CAR.TESLA_MODEL_3, fingerprint, [], False, False, False)
      assert CP.radarUnavailable != radar

  def test_no_radar_car(self):
    # Model X doesn't have radar DBC defined, should always be unavailable
    for radar in (True, False):
      fingerprint = gen_empty_fingerprint()
      if radar:
        fingerprint[1][RADAR_START_ADDR] = 8
      CP = CarInterface.get_params(CAR.TESLA_MODEL_X, fingerprint, [], False, False, False)
      assert CP.radarUnavailable  # Always unavailable since no radar DBC


class TestTeslaLongitudinal(unittest.TestCase):
  def test_das_jerk_limits_stock(self):
    # Gentle and hard accels both get the stock DAS jerk limits
    CP = CarInterface.get_params(CAR.TESLA_MODEL_3, gen_empty_fingerprint(), [], True, False, False)
    assert CP.openpilotLongitudinalControl
    CS = SimpleNamespace(out=structs.CarState(vEgo=25., vEgoRaw=25.).as_reader())
    controller = CarController(DBC[CP.carFingerprint], CP)
    parser = CANParser(DBC[CP.carFingerprint][Bus.party], [("DAS_control", 0)], CANBUS.party)

    for accel in (CarControllerParams.ACCEL_MAX, 0.5, 0., -0.2, -0.5, -0.85, -1.5, CarControllerParams.ACCEL_MIN):
      CC = structs.CarControl(enabled=True, longActive=True, actuators=structs.CarControl.Actuators(accel=accel)).as_reader()
      for _ in range(4):
        t = controller.frame * DT_CTRL * 1e9
        _, can_sends = controller.update(CC, CS, t)
        parser.update([(int(t), [m for m in can_sends if m[0] == 0x2b9])])
      assert parser.vl["DAS_control"]["DAS_accelMin"] == pytest.approx(accel, abs=0.05)
      assert parser.vl["DAS_control"]["DAS_jerkMin"] == pytest.approx(CarControllerParams.JERK_LIMIT_MIN, abs=0.02)
      assert parser.vl["DAS_control"]["DAS_jerkMax"] == pytest.approx(CarControllerParams.JERK_LIMIT_MAX, abs=0.02)
