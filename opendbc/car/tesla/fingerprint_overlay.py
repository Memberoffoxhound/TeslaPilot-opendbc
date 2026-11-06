"""Local EPS firmware additions; shipped recognition always takes priority."""
import json
import os
import re
import tempfile
from pathlib import Path

from opendbc.car.fingerprints import FW_VERSIONS
from opendbc.car.fw_versions import build_fw_dict, match_fw_to_car_exact
from opendbc.car.tesla.values import CAR, Ecu, FW_QUERY_CONFIG

PATH = Path("/data/teslapilot/fingerprints/tesla-fw-v1.json")
EPS = (Ecu.eps, 0x730, None)
PLATFORMS = (CAR.TESLA_MODEL_3, CAR.TESLA_MODEL_Y)


def catalog():
  return sorted({(int(y), str(p), d.model) for p in PLATFORMS for d in p.config.car_docs for y in d.year_list}, reverse=True)


def validate(data):
  if not isinstance(data, dict) or not set(data) <= set(PLATFORMS):
    raise ValueError("invalid platform")
  result = {}
  for platform, versions in data.items():
    if not isinstance(versions, list) or len(versions) > 64:
      raise ValueError("invalid versions")
    for text in versions:
      if not isinstance(text, str) or not 2 <= len(text) <= 256:
        raise ValueError("invalid firmware")
      fw = bytes.fromhex(text)
      if re.fullmatch(FW_QUERY_CONFIG.fw_version_regex, fw) is None:
        raise ValueError("invalid EPS reply")
      if any(fw in ecus.get(EPS, []) for p, ecus in FW_VERSIONS.items() if p != platform):
        raise ValueError("conflicting firmware")
      result.setdefault(platform, {EPS: []})[EPS].append(fw)
  for platform, ecus in result.items():
    versions = ecus[EPS]
    if any(fw in other[EPS] for p, other in result.items() if p != platform for fw in versions):
      raise ValueError("ambiguous local firmware")
    ecus[EPS] = sorted(set(versions))
  return result


def load(path=PATH):
  try:
    if path.stat().st_size > 16384:
      return {}
    return validate(json.loads(path.read_text()))
  except (OSError, ValueError, TypeError):
    return {}


def match_local(car_fw, path=PATH):
  extra = load(path)
  return match_fw_to_car_exact(build_fw_dict(car_fw, "tesla"), "tesla", extra_fw_versions=extra) if extra else set()


def save(platform, car_fw, path=PATH):
  extra = load(path)
  versions = [f.fwVersion.hex() for f in car_fw if f.brand == "tesla" and not f.logging and
              (f.ecu, f.address, f.subAddress or None) == EPS]
  if platform not in PLATFORMS or not versions:
    raise ValueError("No fresh Tesla EPS firmware captured; check the harness and retry.")
  data = {p: [v.hex() for v in e[EPS]] for p, e in extra.items()}
  data[platform] = sorted(set(data.get(platform, []) + versions))
  merged = validate(data)
  if match_fw_to_car_exact(build_fw_dict(car_fw, "tesla"), "tesla", extra_fw_versions=merged) != {platform}:
    raise ValueError("Firmware does not uniquely identify this platform.")
  if len(json.dumps(data)) > 16384:
    raise ValueError("Local fingerprint database is full.")
  path.parent.mkdir(parents=True, exist_ok=True)
  fd, tmp = tempfile.mkstemp(dir=path.parent)
  try:
    with os.fdopen(fd, "w") as f:
      json.dump(data, f, sort_keys=True)
      f.flush()
      os.fsync(f.fileno())
    os.replace(tmp, path)
  finally:
    Path(tmp).unlink(missing_ok=True)
