"""Build the AGV host simulator with optional -D overrides."""

import os
import re
import subprocess
import sys
from pathlib import Path
from tempfile import TemporaryDirectory


def main():
    if len(sys.argv) < 2:
        sys.exit("Usage: python build.py OUTPUT [-DKEY=VALUE ...]")

    sim_dir = Path(__file__).resolve().parent
    root = sim_dir.parents[2]
    output = Path(sys.argv[1]).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)

    sources = [
        "BSP/Encoder/encoder.c",
        "App/PID/pid.c",
        "App/Motor_Model/motor_model.c",
        "App/Wheel_Control/wheel_control.c",
        "App/Motion_Profile/motion_profile.c",
        "App/Line_Tracking/line_tracking.c",
        "App/AGV_Motion/agv_motion.c",
        "App/Load_Detect/load_detect.c",
        "App/AGV_Mission/agv_mission.c",
    ]
    include_dirs = ["BSP/Motor", "BSP/Line_Sensor", "BSP/UART", "BSP/Color_Sensor"]
    include_dirs.extend(str(Path(source).parent) for source in sources)

    config = root / "App/Robot_Config/robot_config.h"
    lines = []
    for line in config.read_text(encoding="utf-8").splitlines():
        match = re.match(r"#define\s+(\w+)\s+\S", line)
        if match and not line.rstrip().endswith("\\"):
            lines.extend((f"#ifndef {match[1]}", line, "#endif"))
        else:
            lines.append(line)

    with TemporaryDirectory(dir=output.parent) as generated:
        (Path(generated) / config.name).write_text("\n".join(lines) + "\n", encoding="utf-8")
        command = [
            os.environ.get("CC", "gcc"), "-O2", "-g", "-Wall", "-std=gnu11",
            *sys.argv[2:], "-o", str(output),
            "-I", generated, "-I", str(sim_dir / "stub"),
        ]
        for directory in include_dirs:
            command.extend(("-I", str(root / directory)))
        command.extend((str(sim_dir / "sim.c"), *(str(root / source) for source in sources), "-lm"))
        return subprocess.call(command)


if __name__ == "__main__":
    sys.exit(main())
