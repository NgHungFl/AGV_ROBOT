#!/bin/bash
# Mo phong may tinh: chay CHINH code C cua robot voi mo hinh xe + sa ban de 2.
# Dung:  ./build.sh sim_run [-DLYAP_K=8.0f ...]
#        ./sim_run [blue] [dark] [nomm] [seed=N] [scale=1.01] [L=100] [W=130] [kl=9.6] [kr=11.2] [csv=out.csv] [v]
HERE="$(cd "$(dirname "$0")" && pwd)"
W="$HERE/../.."
OUT=$1; shift
mkdir -p "$HERE/gen"
python3 - <<PY
import re
s=open("$W/App/Robot_Config/robot_config.h").read().split("\n")
o=[]
for l in s:
    m=re.match(r"#define\s+(\w+)\s+(.*)",l)
    if m and not l.rstrip().endswith("\\\\") and not m.group(1).endswith("_H"):
        o+=["#ifndef "+m.group(1),l,"#endif"]
    else: o.append(l)
open("$HERE/gen/robot_config.h","w").write("\n".join(o))
PY
gcc -O1 -g -Wall -std=gnu11 "$@" -o "$OUT" \
 -I"$HERE/gen" -I"$HERE/stub" -I$W/BSP/Motor -I$W/BSP/Encoder -I$W/BSP/Line_Sensor -I$W/BSP/UART -I$W/BSP/Color_Sensor \
 -I$W/App/PID -I$W/App/Motor_Model -I$W/App/Wheel_Control -I$W/App/Motion_Profile -I$W/App/Line_Tracking -I$W/App/AGV_Motion -I$W/App/Load_Detect -I$W/App/AGV_Mission \
 "$HERE/sim.c" $W/BSP/Encoder/encoder.c $W/App/PID/pid.c $W/App/Motor_Model/motor_model.c $W/App/Wheel_Control/wheel_control.c $W/App/Motion_Profile/motion_profile.c \
 $W/App/Line_Tracking/line_tracking.c $W/App/AGV_Motion/agv_motion.c $W/App/Load_Detect/load_detect.c $W/App/AGV_Mission/agv_mission.c -lm
