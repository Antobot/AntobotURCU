# antobot_urcu

Contains scripts for monitoring and managing core uRCU parameters
- launchManager.py: creates standard methods for launching ROS nodes and launchfiles from within other scripts
- softShutdown.py: monitors and carries out requests to shutdown the uRCU system
- urcuMonitor.py: monitors data important to uRCU function (temperature, CPU load, storage, battery level, etc.)