#!/bin/bash
set -m

export CARLA_ROOT=/opt/carla-simulator

export PYTHONPATH=${CARLA_ROOT}/PythonAPI/carla/dist/carla-0.9.11-py3.7-linux-x86_64.egg
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI/carla
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI


${CARLA_ROOT}/CarlaUE4.sh


