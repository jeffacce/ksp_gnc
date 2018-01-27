# Kerbal Space Program GNC scripts
## Python autopilot
Interfaces with KSP with the krpc mod. The new python autopilot can be found [here](https://github.com/jeffacce/ksp_gnc/python). Uses daisy-chained PID controllers to control attitude and speed.

The python autopilot is meant for airplane control: it prioritizes pitch and roll instead of pitch and yaw to change vessel heading.

Features most of the autopilot control options found on a typical airliner A/P panel. Tunes the PIDs on the fly using current dynamic pressure.
