#!/usr/bin/env python3
import math
import sys
import yaml
import struct

REQUIRED_POSITIVE = ("PREDICT_TIME", "SIM_PERIOD", "ANGLE_RESOLUTION", "OBS_RANGE",
                     "MAX_VELOCITY", "MAX_YAWRATE", "MAX_ACCELERATION",
                     "MAX_DECELERATION", "MAX_D_YAWRATE")

def validate(dwa, adapter):
    errors = []
    for key in REQUIRED_POSITIVE:
        value = dwa.get(key)
        if not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            errors.append("%s must be finite and positive" % key)
    for key, minimum in (("VELOCITY_SAMPLES", 2), ("YAWRATE_SAMPLES", 2),
                         ("SIM_TIME_SAMPLES", 1)):
        if type(dwa.get(key)) is not int or not minimum <= dwa[key] <= 1000:
            errors.append("%s must be an integer >= %d" % (key, minimum))
    if isinstance(dwa.get("VELOCITY_SAMPLES"), int) and isinstance(dwa.get("YAWRATE_SAMPLES"), int) and dwa["VELOCITY_SAMPLES"] * (dwa["YAWRATE_SAMPLES"]+1) > 100000:
        errors.append("trajectory sample resource limit exceeded")
    for key, default in (("HZ", 20.0), ("ROBOT_RADIUS", None), ("GOAL_THRESHOLD", None)):
        value = dwa.get(key, default)
        if not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            errors.append("%s must be finite and positive" % key)
    for key in ("MIN_YAWRATE", "MIN_IN_PLACE_YAWRATE", "MAX_IN_PLACE_YAWRATE", "FOOTPRINT_PADDING",
                "OBSTACLE_COST_GAIN", "TO_GOAL_COST_GAIN", "SPEED_COST_GAIN", "PATH_COST_GAIN"):
        value = dwa.get(key)
        if not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0:
            errors.append("%s must be finite and nonnegative" % key)
    if all(isinstance(dwa.get(k), (int,float)) for k in ("MIN_IN_PLACE_YAWRATE","MAX_IN_PLACE_YAWRATE")) and dwa["MIN_IN_PLACE_YAWRATE"] > dwa["MAX_IN_PLACE_YAWRATE"]:
        errors.append("in-place yawrate bounds are inverted")
    low, target, high = (dwa.get(k) for k in ("MIN_VELOCITY", "TARGET_VELOCITY", "MAX_VELOCITY"))
    if not all(isinstance(x, (int, float)) and math.isfinite(x) for x in (low, target, high)) or not low <= target <= high:
        errors.append("MIN_VELOCITY <= TARGET_VELOCITY <= MAX_VELOCITY is required")
    resolution = adapter.get("map_resolution")
    sx, sy = adapter.get("map_size_x"), adapter.get("map_size_y")
    if not all(isinstance(x, (int, float)) and math.isfinite(x) and x > 0 for x in (resolution, sx, sy)):
        errors.append("map dimensions and resolution must be finite and positive")
    elif math.ceil(sx / resolution) * math.ceil(sy / resolution) > 4000000:
        errors.append("local map exceeds 4,000,000 cells")
    increment = adapter.get("scan_angle_increment")
    angle_resolution = dwa.get("ANGLE_RESOLUTION")
    if not isinstance(increment, (int, float)) or not math.isfinite(increment) or increment <= 0:
        errors.append("scan_angle_increment must be finite and positive")
    else:
        try:
            actual_increment = struct.unpack('f', struct.pack('f', increment))[0]
            if not math.isfinite(actual_increment) or actual_increment <= 0:
                errors.append("scan increment is not representable as positive float32")
            elif isinstance(angle_resolution, (int,float)) and math.isfinite(angle_resolution) and int(angle_resolution/actual_increment) < 1:
                errors.append("ANGLE_RESOLUTION / scan_angle_increment must be >= 1")
        except (OverflowError, struct.error):
            errors.append("scan increment is not representable as float32")
    if not isinstance(dwa.get("ROBOT_FRAME"), str) or not dwa["ROBOT_FRAME"].strip():
        errors.append("ROBOT_FRAME is required")
    if adapter.get("target_frame", dwa.get("ROBOT_FRAME")) != dwa.get("ROBOT_FRAME"):
        errors.append("adapter target_frame and ROBOT_FRAME must match")
    return errors

def main():
    if len(sys.argv) != 3:
        print("usage: validate_integration_config.py DWA_YAML ADAPTER_YAML", file=sys.stderr)
        return 2
    try:
        with open(sys.argv[1], encoding="utf-8") as stream:
            dwa = yaml.safe_load(stream) or {}
        with open(sys.argv[2], encoding="utf-8") as stream:
            adapter = yaml.safe_load(stream) or {}
        if not isinstance(dwa, dict) or not isinstance(adapter, dict):
            raise ValueError("configuration root must be a mapping")
    except (OSError, yaml.YAMLError, ValueError) as error:
        print("configuration error: "+str(error),file=sys.stderr)
        return 1
    errors = validate(dwa, adapter)
    for error in errors:
        print("configuration error: " + error, file=sys.stderr)
    return 1 if errors else 0

if __name__ == "__main__":
    sys.exit(main())
