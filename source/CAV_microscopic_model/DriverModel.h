/*==========================================================================*/
/*  DriverModel.h                                    DLL Module for VISSIM  */
/*                                                                          */
/*  Interface module for external driver models.                            */
/*                                                                          */
/*  Version of 2010-06-17                                   Lukas Kautzsch  */
/*==========================================================================*/

#ifndef __DRIVERMODEL_H
#define __DRIVERMODEL_H

#ifndef _CONSOLE
#include <windows.h>
#endif

/* In the creation of DriverModel.DLL all files must be compiled */
/* with the preprocessor definition DRIVERMODEL_EXPORTS.         */
/* Programs that use DriverModel.DLL must not be compiled        */
/* with that preprocessor definition.                            */

#ifdef DRIVERMODEL_EXPORTS
#define DRIVERMODEL_API extern "C" __declspec(dllexport)
#else
#define DRIVERMODEL_API extern "C" __declspec(dllimport)
#endif

/*==========================================================================*/

/* general data: */
/* (index1, index2 irrelevant) */
#define  DRIVER_DATA_PATH                    101
           /* string: absolute path to the model's data files directory */
#define  DRIVER_DATA_TIMESTEP                102
           /* double: simulation time step length [s] */
#define  DRIVER_DATA_TIME                    103
           /* double: current simulation time [s] */
#define  DRIVER_DATA_PARAMETERFILE           104
           /* string: name (including absolute path) of a vehicle type's parameter file */
#define  DRIVER_DATA_STATUS                  105
           /* long:                                                           
           /* 0=OK,                                                             */
           /* 1=Info (there's some further information available),              */
           /* 2=Warning (there are warnings available),                         */
           /* 3=Error (an recoverable error occured but simulation can go on),  */
           /* 4=Heavy (an unrecoverable error occured and simulation must stop) */
           /* (used by DriverModelGetValue()!)                                  */
#define  DRIVER_DATA_STATUS_DETAILS          106
           /* string: XML format according to the schema file VDMStatus.xsd  */
           /* (used by DriverModelGetValue()!)                               */
           /* (is retrieved by VISSIM only if DRIVER_DATA_STATUS is nonzero) */

/* current vehicle driver unit data (VDU to be moved next): */
/* (index1, index2 irrelevant) */
#define  DRIVER_DATA_VEH_ID                  201
           /* long:   vehicle number */
#define  DRIVER_DATA_VEH_LANE                202
           /* long:   current lane number (rightmost = 1) */
#define  DRIVER_DATA_VEH_ODOMETER            203
           /* double: total elapsed distance in the network [m] */
#define  DRIVER_DATA_VEH_LANE_ANGLE          204
           /* double: angle relative to the middle of the lane [rad] */
           /*         (positive = turning left)                      */
#define  DRIVER_DATA_VEH_LATERAL_POSITION    205
           /* double: distance of the front end from the middle of the lane [m] */
           /*         (positive = left of the middle, negative = right)         */
#define  DRIVER_DATA_VEH_VELOCITY            206
           /* double: current speed [m/s] */
#define  DRIVER_DATA_VEH_ACCELERATION        207
           /* double: current acceleration [m/s²] */
#define  DRIVER_DATA_VEH_LENGTH              208
           /* double: vehicle length [m] */
#define  DRIVER_DATA_VEH_WIDTH               209
           /* double: vehicle width [m] */
#define  DRIVER_DATA_VEH_WEIGHT              210
           /* double: vehicle weight [kg] */
#define  DRIVER_DATA_VEH_MAX_ACCELERATION    211
           /* double: maximum possible acceleration [m/s²] */
           /*         (depending on current speed)         */
#define  DRIVER_DATA_VEH_TURNING_INDICATOR   212
           /* long:   left = 1, right = -1, none = 0, both = 2 */
           /*         (also used by DriverModelGetValue()!) */
#define  DRIVER_DATA_VEH_CATEGORY            213
           /* long:   car = 1, truck = 2, bus = 3, tram = 4, */
           /*         pedestrian = 5, bike = 6               */
#define  DRIVER_DATA_VEH_PREFERRED_REL_LANE  214
           /* long:   positive = left, 0 = current lane, negative = right */
#define  DRIVER_DATA_VEH_USE_PREFERRED_LANE  215
           /* long:   0 = only preferable (e.g. European highway) */
           /*         1 = necessary (e.g. before a connector)     */
#define  DRIVER_DATA_VEH_DESIRED_VELOCITY    216
           /* double: desired speed [m/s]              */
           /*         (also used by DriverModelGetValue()!) */
#define  DRIVER_DATA_VEH_X_COORDINATE        217
           /* double: world coordinate X (vehicle front end) */
#define  DRIVER_DATA_VEH_Y_COORDINATE        218
           /* double: world coordinate Y (vehicle front end) */
#define  DRIVER_DATA_VEH_TYPE                219
           /* long:   vehicle type number (user defined) */
#define DRIVER_DATA_VEH_COLOR                220
           /* long:   vehicle color (24 bit RGB value) */
           /*         (also used by DriverModelGetValue()!) */
#define DRIVER_DATA_VEH_CURRENT_LINK         221
           /* long:   current link number */
#define DRIVER_DATA_VEH_NEXT_LINKS           222
           /* long:   following link number(s) of the vehicle's route        */
           /* This message is sent from VISSIM only if DriverModelSetValue() */
           /* returned 1 for DRIVER_DATA_VEH_CURRENT_LINK.                   */
           /* It is sent once for each link in the route.                    */
#define  DRIVER_DATA_VEH_ACTIVE_LANE_CHANGE  223
           /* long:   direction of an active lane change movement */
           /*         (+1 = to the left, 0 = none, -1 = to the right) */
#define  DRIVER_DATA_VEH_REL_TARGET_LANE     224
           /* long:   target lange */
           /*         (+1 = next one left, 0 = current lane, -1 = next one right) */
             
/* nearby vehicle driver unit data: */
/* (index1 = relative lane: +2 = second to the left, +1 = next one to the left,   */
/*                           0 = current lane,                                    */
/*                          -1 = next one to the right, -2 = second to the right) */
/* (index2 = relative position: positive = downstream (+1 next, +2 second next)   */
/*                              negative = upstream (-1 next, -2 second next))    */
#define  DRIVER_DATA_NVEH_ID                 301
           /* long:   vehicle number */
           /*         (negative = no vehicle at this lane/position) */
#define  DRIVER_DATA_NVEH_LANE_ANGLE         302
           /* double: angle relative to the middle of the lane [rad] */
           /*         (positive = turning left)                      */
#define  DRIVER_DATA_NVEH_LATERAL_POSITION   303
           /* double: distance of the front end from the middle of the lane [m] */
           /*         (positive = left of the middle, negative = right)         */
#define  DRIVER_DATA_NVEH_DISTANCE           304
           /* double: gross distance [m] (front end to front end) */
#define  DRIVER_DATA_NVEH_REL_VELOCITY       305
           /* double: speed difference [m/s] (veh. speed - nveh. speed) */
#define  DRIVER_DATA_NVEH_ACCELERATION       306
           /* double: current acceleration [m/s²] */
#define  DRIVER_DATA_NVEH_LENGTH             307
           /* double: vehicle length [m] */
#define  DRIVER_DATA_NVEH_WIDTH              308
           /* double: vehicle width [m] */
#define  DRIVER_DATA_NVEH_WEIGHT             309
           /* double: vehicle weight [kg] */
#define  DRIVER_DATA_NVEH_TURNING_INDICATOR  310
           /* long:   left = 1, right = -1, none = 0, both = 2 */
#define  DRIVER_DATA_NVEH_CATEGORY           311
           /* long:   car = 1, truck = 2, bus = 3, tram = 4, */
           /*         pedestrian = 5, bike = 6               */
#define  DRIVER_DATA_NVEH_LANE_CHANGE        312
           /* long:   direction of a current lane change              */
           /*         (+1 = to the left, 0 = none, -1 = to the right) */

/* link data: */
/* (index1, index2 irrelevant) */
#define  DRIVER_DATA_NO_OF_LANES             401

/* lane data: */
/* (index1 = lane number: rightmost = 1) */
/* (index2 irrelevant) */
#define  DRIVER_DATA_LANE_WIDTH              501
           /* double: lane width [m] */
#define  DRIVER_DATA_LANE_END_DISTANCE       502
           /* distance to end of lane [m] */
           /* (can be emergency stop position before connector) */
           /* (negative = no end of lane in visibility range) */

/* environment data: */
/* (index1, index2 irrelevant) */
#define  DRIVER_DATA_RADIUS                  601
           /* double: current curve radius [m] */
#define  DRIVER_DATA_MIN_RADIUS              602
           /* double: minimum curve radius [m] in visibility range */
#define  DRIVER_DATA_DIST_TO_MIN_RADIUS      603
           /* double: distance [m] to spot of minimum curve radius */
#define  DRIVER_DATA_SLOPE                   604
           /* double: current slope (negative = drop) */
           /* (e.g. -0.026 = -2.6%)                   */
#define  DRIVER_DATA_SLOPE_AHEAD             605
           /* double: slope at end of visibility range */
           /* (negative = drop)                        */

/* traffic sign data: */
/* (index1, index2 irrelevant) */
#define  DRIVER_DATA_SIGNAL_DISTANCE         701
           /* double: distance [m] to next signal head */
           /* (negative = no signal head visible)      */
#define  DRIVER_DATA_SIGNAL_STATE            702
           /* long:   red = 1, amber = 2, green = 3, red/amber = 4, */
           /*         amber flashing = 5, off = 6, green arrow = 7  */
#define  DRIVER_DATA_SIGNAL_STATE_START      703
           /* double: simulation time [s] when signal changed to current state */
#define  DRIVER_DATA_SPEED_LIMIT_DISTANCE    704
           /* double: distance [m] to "speed limit sign" */
           /*         (reduced speed area: real distance) */
           /*         (desired speed decision: 1.0 m when just passed) */
           /*         (negative: no sign visible) */
#define  DRIVER_DATA_SPEED_LIMIT_VALUE       705
           /* double: speed limit [km/h] */
           /*         (0 = end of reduced speed area) */

/* driving behaviour data: */
/* (index1, index2 irrelevant) */
/* (must be provided by the driver model after DRIVER_COMMAND_MOVE_DRIVER) */
/* (usually for GetValue(), but will be set by SetValue() as suggestion    */
/*  if GetValue (DRIVER_DATA_WANTS_SUGGESTION, ...) delivers 1)            */
#define  DRIVER_DATA_WANTS_SUGGESTION        801
           /* long:   flag: does driver model want suggestion? */
           /*         (1 = yes, 0 = no) */
#define  DRIVER_DATA_DESIRED_ACCELERATION    802
           /* double: desired acceleration [m/s²] in next time step */
#define  DRIVER_DATA_DESIRED_LANE_ANGLE      803
           /* double: desired angle relative to the middle of the lane [rad] */
           /*         (positive = turning left) */
#define  DRIVER_DATA_ACTIVE_LANE_CHANGE      804
           /* long:   direction of an active lane change movement */
           /*         (+1 = to the left, 0 = none, -1 = to the right) */
           /*         (must be != 0 while lane change is not completed) */
           /*         (will be used for NVEH_LANE_CHANGE) */
#define  DRIVER_DATA_REL_TARGET_LANE         805
           /* long:   target lange */
           /*         (+1 = next one left, 0 = current lane, -1 = next one right) */
#define  DRIVER_DATA_SIMPLE_LANECHANGE       806
           /* long:   flag: does driver model want VISSIM to control the lateral  */
           /*         movement during the lane change (i.e. start lane change     */
           /*         when ACTIVE_LANE_CHANGE != 0 but ignore DESIRED_LANE_ANGLE) */
           /*         and stop the lane change after the vehicle has reached the  */
           /*         middle of the new lane?                                     */
           /*         (1 = yes, 0 = no)                                           */

/*--------------------------------------------------------------------------*/

double GetDesiredAcc(double d,
					 double a1, 
					 double v1,
					 double a2,
					 double v2, 
					 long lvid,
					 double H,
					 long lvid2,
					 double a3,
					 double v3,
					 double d2);

int ControlVehicle();
int SafetyMeassure();
int InitArrays();
double GetLateralPos(double lateralpos); 

DRIVERMODEL_API  int  DriverModelSetValue (long   type, 
                                           long   index1,
                                           long   index2,
                                           long   long_value,
                                           double double_value,
                                           char   *string_value);

/* Sets the value of a data object of type <type>, selected by <index1> */
/* and possibly <index2>, to <long_value>, <double_value> or            */
/* <*string_value> (object and value selection depending on <type>).    */
/* Return value is 1 on success, otherwise 0.                           */

/*--------------------------------------------------------------------------*/

DRIVERMODEL_API  int  DriverModelGetValue (long   type, 
                                           long   index1,
                                           long   index2,
                                           long   *long_value,
                                           double *double_value,
                                           char   **string_value);

/* Gets the value of a data object of type <type>, selected by <index1> */
/* and possibly <index2>, and writes that value to <*double_value>,     */
/* <*float_value> or <**string_value> (object and value selection       */
/* depending on <type>).                                                */
/* Return value is 1 on success, otherwise 0.                           */

/*==========================================================================*/

#define  DRIVER_COMMAND_INIT            0
           /* called from VISSIM once at the start of a simulation run */
           /* values set before: DRIVER_DATA_PATH     */
           /*                    DRIVER_DATA_TIMESTEP */
           /*                    DRIVER_DATA_TIME     */
           /* values got after:  DRIVER_DATA_WANTS_SUGGESTION     */
           /*                    DRIVER_DATA_AUTOMATIC_LANECHANGE */

#define  DRIVER_COMMAND_CREATE_DRIVER   1
           /* called from VISSIM once per vehicle entering the network */
           /* values set before: DRIVER_DATA_VEH_ID               */
           /*                    DRIVER_DATA_VEH_DESIRED_VELOCITY */

#define  DRIVER_COMMAND_KILL_DRIVER     2
           /* called from VISSIM once per vehicle leaving the network */
           /* value set before: DRIVER_DATA_VEH_ID */

#define  DRIVER_COMMAND_MOVE_DRIVER     3
           /* called from VISSIM once per time step during a simulation run */
           /* values set before: all values                      */
           /*                    (driving behaviour data only if */
           /*                     DRIVER_DATA_WANTS_SUGGESTION)  */

/*--------------------------------------------------------------------------*/

DRIVERMODEL_API  int  DriverModelExecuteCommand (long number);

/* Executes the command <number> if that is available in the driver */
/* module. Return value is 1 on success, otherwise 0.               */

/*==========================================================================*/

#endif /* __DRIVERMODEL_H */

/*==========================================================================*/
/*  Ende of DriverModel.h                                                   */
/*==========================================================================*/
