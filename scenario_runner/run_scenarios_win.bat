call setup_env_win.bat

set SCENARIO_NAME=MonitorSurroundingsScenario
set SCENARIO_FILE=monitor_surroundings_scenario

python scenario_runner.py ^
--scenario %SCENARIO_NAME% ^
--reloadWorld ^
--output ^
--additionalScenario="%SCENARIO_DIR%\%SCENARIO_FILE%.py" ^
--configFile="%SCENARIO_DIR%\%SCENARIO_NAME%.xml"


::--debug ^
::--file ^
::--outputDir=%BASE_DIR% ^