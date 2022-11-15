set BASE_DIR=%~dp0..\
set SCENARIO_DIR=%BASE_DIR%\carla_scenarios
set MONITOR_DIR=%BASE_DIR%\carla_monitors
set CARLA_ROOT=D:\github\CARLA_0.9.12
set SCENARIO_RUNNER_ROOT=D:\github\scenario_runner
set PYTHONPATH=%PYTHONPATH%;%CARLA_ROOT%\PythonAPI\carla\dist\carla-0.9.12-py3.7-win-amd64.egg
set PYTHONPATH=%PYTHONPATH%;%CARLA_ROOT%\PythonAPI\carla
set PYTHONPATH=%PYTHONPATH%;%BASE_DIR%

cd %SCENARIO_RUNNER_ROOT%