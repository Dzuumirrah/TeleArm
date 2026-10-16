#!/bin/bash
# Host-side build of the math libraries + unit tests (no ESP32 needed).
set -eo pipefail
cd "$(dirname "$0")/../.."
LIB=${MPU_LIB:-$HOME/MPU9250_WE/src}
mkdir -p /tmp/telearm_build
FLAGS="-std=gnu++11 -DARDUINO=10819 -O2 -Wall -Wextra -Wno-unused-parameter -Wno-missing-field-initializers -Itests/host/shim -I$LIB -Ilib/configs -Ilib/kinematics -Ilib/connections -Ilib/session -Itests/host"
SRC="tests/host/shim/shim.cpp $LIB/xyzFloat.cpp lib/configs/configurations.cpp lib/kinematics/kinematic.cpp lib/kinematics/madgwick.cpp lib/kinematics/calibration.cpp lib/kinematics/cartesian.cpp"
g++ $FLAGS $SRC tests/host/test_kinematics.cpp -o /tmp/telearm_build/test_kinematics -lm
/tmp/telearm_build/test_kinematics

# ---- firmware layers compiled for real against the shim (type check) + simulated end-to-end test ----
SIM_SRC="tests/host/shim/shim.cpp $LIB/xyzFloat.cpp lib/configs/configurations.cpp lib/kinematics/kinematic.cpp lib/kinematics/madgwick.cpp lib/kinematics/calibration.cpp lib/kinematics/cartesian.cpp lib/connections/storage.cpp lib/connections/networks.cpp lib/session/session.cpp tests/host/sim_components.cpp"
for f in lib/connections/components.cpp src/main.cpp src/slave_rx_test/slave_rx_test.cpp $LIB/MPU6500_WE.cpp $LIB/MPU9250_WE.cpp; do g++ $FLAGS -fsyntax-only $f; done
g++ $FLAGS $SIM_SRC tests/host/test_session.cpp -o /tmp/telearm_build/test_session -lm
/tmp/telearm_build/test_session | grep -v '^[0-9]*,'    # the 10 Hz debug CSV is filtered out

# Optional: forearm heading-drift experiment, hinge correction OFF vs ON (DRIFT=1 ./tests/host/build_and_run.sh)
if [ -n "$DRIFT" ]; then
  sed 's/USE_HINGE_YAW_CORRECTION = false/USE_HINGE_YAW_CORRECTION = true/' lib/configs/configurations.cpp > /tmp/telearm_build/configurations_hinge.cpp
  g++ $FLAGS ${SIM_SRC/lib\/configs\/configurations.cpp//tmp/telearm_build/configurations_hinge.cpp} tests/host/test_session.cpp -o /tmp/telearm_build/test_session_hinge -lm
  /tmp/telearm_build/test_session drift | grep -v '^[0-9]*,\|^\[CAL\]\|^\[NOW\]\|^\[SESSION\]'
  /tmp/telearm_build/test_session_hinge drift | grep -v '^[0-9]*,\|^\[CAL\]\|^\[NOW\]\|^\[SESSION\]'
fi
