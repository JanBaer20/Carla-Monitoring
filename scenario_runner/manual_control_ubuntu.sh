#!/bin/bash

# load paths
THIS_FILE=`readlink -f "$0"`
THIS_DIRECTORY=`dirname $THIS_FILE`
source $THIS_DIRECTORY/setup_env_ubuntu.sh

# start scenario runner
# python3.7 scenario_runner.py --scenario CutInFrom_left_Lane --reloadWorld --output --json &
/usr/bin/python3.7 $SCENARIO_RUNNER_ROOT/manual_control.py