#!/bin/bash

# get the directory of this script for relative paths to other files from git repository
THIS_FILE=`readlink -f "$0"`
echo "THIS_FILE: $THIS_FILE"
export CARLASIM_ROOT=$(realpath "$(dirname $THIS_FILE)/..")
echo "CARLASIM_ROOT: $CARLASIM_ROOT"

# These default values are the settings for elixir. Export them in your .bashrc to override default values
#
# DON'T MAKE ANY CHANGES HERE
#
[[ -z "${CARLA_ROOT}" ]] && export CARLA_ROOT="/opt/carla-simulator"
echo "CARLA_ROOT: $CARLA_ROOT"

[[ -z "${SCENARIO_RUNNER_ROOT}" ]] && export SCENARIO_RUNNER_ROOT="/opt/scenario_runner"
echo "SCENARIO_RUNNER_ROOT: $SCENARIO_RUNNER_ROOT"

[[ -z "${CARLA_HOSTNAME}" ]] && export CARLA_HOSTNAME="localhost"
echo "CARLA_HOSTNAME: $CARLA_HOSTNAME"

# some opts possibilities for your .zshrc
# -opengl
# -quality-level=Low
# -quality-level=Epic
[[ -z "${CARLA_OPTS}" ]] && export CARLA_OPTS=""
echo "CARLA_OPTS: $CARLA_OPTS"

# generated project paths
#
export SCENARIO_DIR=$CARLASIM_ROOT/carla_scenarios
echo "SCENARIO_DIR: $SCENARIO_DIR"
export MONITOR_DIR=$CARLASIM_ROOT/carla_monitors
echo "MONITOR_DIR: $MONITOR_DIR"

# generated python paths
#
export PYTHONPATH=${CARLA_ROOT}/PythonAPI/carla/dist/carla-0.9.12-py3.7-linux-x86_64.egg
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI/carla
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI
export PYTHONPATH=$PYTHONPATH:${CARLASIM_ROOT}


