31/10/2017
Arduplane has been adapted to control the linear servo and three rotational servos.
Changes are marked in the code with Clara Todd so are searchable.
The bottom of this file gives changes that will have to be made to move the UAV from wind tunnel testing to freefall testing


FILES THAT HAVE BEEN CHANGED AND THE CHANGES

Plane.h:
struct skydiver stores information relevant to skydiver.
LUT generated in 2016 is included but has been compressed to half size for memory reasons

Attitude.cpp:
Added UAV_Yaw_Control to control rotational servo position + linear servo position.

Arduplane.cpp:
Removed stabilize task and replaced with UAV_Yaw_Control
Added get_pixy_block function
Pixy camera, linear servo length and rotational servo trims are initialised

Control_modes.cpp:
Inverted flight channel (channel 6) has been hijacked to allow SWG on controller to reset the value on the integral in the controller to the current heading/pitch angle

defines.h:
Defined additional log message headings

GCS_Mavlink.cpp:
Added case MAVLINK_MSG_ID_GLOBAL_POSITION_INT in handle message to receive GPS position from skydiver

Log.cpp:
Log packages to log GPS of skydiver and Pixy messages. 

Parameters.cpp:
Adjusted flight_mode channels to Guided, Acro, Loiter for different controls of UAV (fully manual, partially autonomous, fully autonomous respectively)
Inverted flight set to channel 6

Servos.cpp:
Removed everything from set_servos and servos_output except what is necessary to pass a value to the servos.

tracking.cpp:
Added file with tracking functions of UAV

Libraries\APM_Control\AP_YawController.cpp:
Added PID controller to control yaw of UAV

Libraries\AP_BoardConfig\AP_BoardConfig.cpp:
Removed need for safety switch

Libraries\AP_Arming\AP_Arming.cpp:
Disabled requirement to arm

Libraries\AP_AHRS\AP_AHRS.cpp:
Set Orientation to Roll 180

AP_IRLockI2C.cpp:
Adjusted read_frames to store pixel positions 

IRLock.cpp:
Added override function: get_unit_vector_body to return pixel position and sizes of Skydiver.


TO CHANGE TO BE SUITABLE FOR FREEFALL TEST:

In Libraries\APM_Control\AP_YawController.cpp:
In PID: Change _ahrs.roll_sensor for _ahrs.yaw_sensor

In Libraries\AP_AHRS\AP_AHRS.cpp:
Change ORIENTATION to (probably) Roll180Pitch270 (but might require some playing around)

Check pixy camera and GPS transition works properly (i.e. both measurements have same sign)
(Beware that UAV steer_rate is given as steer_rate = -skydiver.azimuth*100;)

