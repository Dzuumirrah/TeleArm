#!/bin/bash
# Host-side simulation of the SLAVE firmware (no ESP32 needed):
# real slave_session.cpp + packet code against simulated steppers / switches / AS5600 (sim_actuation.cpp).
#   MPU_LIB=<path to MPU9250_WE/src> ./tests/host/build_and_run_slave.sh
set -eo pipefail
cd "$(dirname "$0")/../.."
LIB=${MPU_LIB:-$HOME/MPU9250_WE/src}
mkdir -p /tmp/telearm_build
FLAGS="-std=gnu++11 -DARDUINO=10819 -O2 -Wall -Wextra -Wno-unused-parameter -Wno-missing-field-initializers -Itests/host/shim -I$LIB -Ilib/configs -Ilib/kinematics -Ilib/connections -Ilib/actuation -Ilib/slave_session -Itests/host"
SRC="tests/host/shim/shim.cpp $LIB/xyzFloat.cpp lib/connections/networks.cpp lib/connections/remotelog.cpp lib/slave_session/slave_session.cpp tests/host/sim_actuation.cpp tests/host/test_slave.cpp"
g++ $FLAGS lib/configs/configurations.cpp $SRC -o /tmp/telearm_build/test_slave -lm
/tmp/telearm_build/test_slave | grep -v '^S '          # the 10 Hz status line is filtered out
# same tests with binary PC output
sed 's/PC_OUTPUT_BINARY    = false/PC_OUTPUT_BINARY    = true/' lib/configs/configurations.cpp > /tmp/telearm_build/configurations_bin.cpp
g++ $FLAGS /tmp/telearm_build/configurations_bin.cpp $SRC -o /tmp/telearm_build/test_slave_bin -lm
/tmp/telearm_build/test_slave_bin
