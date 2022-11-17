#!/bin/bash

# load paths
THIS_FILE=`readlink -f "$0"`
THIS_DIRECTORY=`dirname $THIS_FILE`
source $THIS_DIRECTORY/setup_env_ubuntu.sh

# prefer environment setting, but fallback to MonitorSurroundings in case env is not set
[[ -z "${SCENARIO_NAME}" ]] && export SCENARIO_NAME="PruefungHighwayScenario"
echo "SCENARIO_NAME: $SCENARIO_NAME"
[[ -z "${SCENARIO_FILE}" ]] && export SCENARIO_FILE="pruefung_highway_scenario"
echo "SCENARIO_FILE: $SCENARIO_FILE"

# start carla
# pushd $CARLA_ROOT
#DISPLAY= ./CarlaUE4.sh $CARLA_OPTS &
# ./CarlaUE4.sh -RenderOffScreen $CARLA_OPTS &
# CARLA_PID=$!
# popd

# echo "Waiting for carla to start ..."
# sleep 4

#/usr/bin/python3.7 /home/janbaer/Desktop/manual_control_steeringwheel.py

#sleep 4

# start scenario runner
# python3.7 scenario_runner.py --scenario CutInFrom_left_Lane --reloadWorld --output --json &
/usr/bin/python3.7 $SCENARIO_RUNNER_ROOT/scenario_runner.py --scenario $SCENARIO_NAME --reloadWorld \
   --output --file \
   --outputDir=. \
	 --host=$CARLA_HOSTNAME \
   --additionalScenario="${SCENARIO_DIR}/${SCENARIO_FILE}.py" \
   --debug \
   --configFile="${SCENARIO_DIR}/${SCENARIO_NAME}.xml" 


#  sleep 4

# /usr/bin/python3.7 /home/janbaer/Desktop/manual_control_steeringwheel.py &

#/usr/bin/python3.7 /home/Carla/carlasim/monitors/vehicle_passed_red_traffic_light.py &

## start carla API client to that outputs frame-%08d.png files until simulation ends
#export SDL_VIDEODRIVER=dummy
#FPS=`python3.7 $CARLASIM_ROOT/scenario_runner/headless_autopilot.py --res 800x600 --host $CARLA_HOSTNAME | grep FPS | cut -d ' ' -f2`
#echo "FPS: $FPS"
#
## create video
#rm output.mp4
#echo -n "Rendering output.mp4 from frames... "
#ffmpeg -framerate $FPS -loglevel fatal -i frame-%08d.png -pix_fmt yuv420p output.mp4
#echo "done."
#rm frame-*
#
#echo "Killing Carla"
#pkill Carla
