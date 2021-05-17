/*==========================================================================*/
/*  DriverModel.cpp                                  DLL Module for VISSIM  */
/*                                                                          */
/*  Interface module for external driver models.                            */
/*  Dummy version that simply sends back VISSIM's suggestions to VISSIM.    */
/*                                                                          */
/*  Version of 2010-03-02                                   Lukas Kautzsch  */
/*==========================================================================*/

/* ---------------------------- Revision Note ------------------------------*/  
/* Nov. 2012																*/
/*  CACC Application Enhancement											*/
/*  Modified by Joyoung Lee, University of Virginia.						*/
/*
/* Jul. 2013
/*  CACC Lane changing Model Enhancement 
/*  Modified by Joyoung Lee, University of Virginia
/* 
/* May. 2014
/*  Algorithm Minor revision
/*  Modified by Joyoung Lee, New Jersey Institute of Technology  
/*
/* Nov. 2014
/*  Revised for VISSIM 6 environment, Joyoung Lee NJIT
/*  Note : VISSIM 7 (up to -04) did not work; it kept crashing when the first CACC vehicle reaches at the position of about 80%  of link length.
/* 
/* Dec. 2014 
/*  Revised for the CACC Simulation Manager program, Joyoung Lee NJIT   
/* 
/* Feb. 2015
/* Code Block descriptions (input/output flow) were added. 
/* 
/* May .2019
// 修复了横向位置为0时候可能报错的问题
// 修复了CUT IN JOIN 的时候可能撞车的问题
// 更改了自带的跟驰模型 ： IDM 模型
// 增加了换道的逻辑
//期望速度和速度的单位都是M/S!!!!!!!!!!!!!!!!!!!!!!
/*==========================================================================*/


/*>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> START OF CODE BLOCK #1>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
/*
                                   Primary Input/Output for CODE BLOCK #1
  
  This block primarily conducts the initializations of variables which are from not only VISSIM API defaults but also user-defined one.     
  More specific information of each variable is explained in the User-Guide

  Input : All variables initialized in this block 
  Outout : All variables initialized in this block
  

*/

#include "DriverModel.h"
#include <stdio.h>
#include <iostream> 
#include <fstream>
#include <list>
#include <math.h>
#include <iostream>
#include <ctime>
#include <map>
#include <string> 

using namespace std;

/*==========================================================================*/

double  desired_acceleration = 0.0;
double  desired_lane_angle   = 0.0; // Radian
double	desired_lane_angle_tmp = 0.0;
long    active_lane_Change   = 0;
long    rel_target_lane      = 0;
long	veh_rel_target_lane = 0;
double  desired_velocity     = 0.0;
long    turning_indicator    = 0;
long    vehicle_color        = RGB(0,0,0);

long veh_type = 101;
long current_link = 0;
long current_lane = 0; 
long lanes_current_link = 0; // Dec. 15. 2014
double timestep = 0.0;
long current_veh = 0;
long vissim_suggestion = 0; // 0 indicates no: not listen to VISSIM, 1 otherwise
long simple_lanechange = 0;
long adj_veh;
long adj_veh_class; 
char* dt;
time_t now;
long AdjVehicles[5][5];
double AdjVehiclesWidth[5][5];
double AdjVehiclesSpeed[5][5];
double AdjVehiclesDist[5][5];
double AdjVehiclesAcc[5][5];
long AdjVehiclesLaneChange[5][5];
long AdjVehiclesCurrentLane[5][5];
long AdjVehiclesTargetLane[5][5]; 

ofstream fout;//("out_newdll.txt",std::ios_base::app);
ofstream fout_ncacc;
ofstream fout_SAFETY;
ofstream fout_TotalSafetyResult;
ifstream fin;
string str;
char ch;
double val = 0.0;

map<long,int> VehStatus; // 0: false, >1: true
map<long,double> PreviousVehAcc;
map<long,double> PreviousVehAcc2;
map<long,double> PreviousVehAcc3;
map<long,double> PreviousVehAcc4;
map<long,double> PreviousVehAcc5;
map<long,int> start_Control_Time;//当5秒后才开始延时的策略
map<long,int> VehTargetLane;
map<long,int> VehTag;
map<long,int> VehToAdjustDecForCutIn;
map<long,int> CACCLeader;
map<long,int> HowManyFollwers;

//Safety measurement
map<long,double> TTC;
map<long,double> TET;
map<long,double> TIT;
map<long,int>NCJ;
map<long,double> TERCRI;
double TTC_Total=0.0;
double TET_Total=0.0;
double TIT_Total=0.0;
int NCJ_Total=0;
double TERCRI_Total=0.0; 



long LeadingVeh_ID =0;
long Lv2_ID=0;
double LeadingVeh_Spd = 0.0;
double Lv2_Spd  = 0.0;
double LeadingVeh_Acc = 0.0; 
double Lv2_Acc  = 0.0;
double DistLeadingVeh = 0.0; 
double DistLeadingVeh2 = 0.0; 
double DistLatterVeh=0.0;
double DistTargetLeadingVeh = 0.0; 
//double DistFollowingVehOnTargetLane = 0.0;
//double DistLeadingVehOnTargetLane = 0.0;
//double SpeedFollowingVehOnTargetLane = 0.0;
//double SpeedLeadingVehOnTargetLane = 0.0;

double sim_time = 0.0;
double Spd_vehicle = 0.0;
double Acc_vehicle = 0.0;
double MaxAcc_vehicle = 0.0;
double Headway_current = 0.0;
double Headway_new = 0.0;
double double_tmp =0.0;
double deltaX = 0.0;
long long_tmp = 0;
double lateral_pos = 0.0;
double lateral_pos_ind = 0.0; 
double veh_od = 0.0;
bool WrtFlag = false;
int tlane = 2;

double s1 = 0.0;
double s2 = 0.0;
double d = 0.0;
double dd = 0.0;
double v = 0.0;

double desired_acc_zero = 0.00000001; // m/s/s

double init_acc = 0.0001;
double t_ = 2.5;
double acc_time = 3.0; // 3 sec
double dec_time = 30.0; // 5 sec
double uniformspeed_time = 50.0; 
double MaxSpdRegFactor = 1.15; // allow 120% from the desired speed to catch up the leading vehicle. 
//long simple_lanechange = 0;
int JoinCase = 0;
double MinAllowLCDist = 20.0; // in meter
long VehToWatch =81;
double lane_angle = 0.021;
double VehLengthIndicator = 3.749; // CACC车辆长度是3.749m 必须事先在VISSIM6.0中设置好车辆模型，否则会把CACC认成非CACC

//-- Default setting for adjustable variables. 
double Headway = 0.75; //ADJUSTABLE
double Headway_No = 1.2; //ADJUSTABLE
double MaxLookAheadDist = 300.0; // in meter //ADJUSTABLE
double MInCACCSpdBound = 30; // m/s //ADJUSTABLE
int MinFollower = 4; //最少队列车辆数
int MaxFollower = 8;//最多队列车辆数
double MaxSpdForCACC = 36.0; //ADJUSTABLE
double desired_dec_inc = -2.88; //ADJUSTABLE 
double MaxDeceleration = -9.0; //ADJUSTABLE
double desired_dec_inc_join = -1.5; //ADJUSTABLE
double desired_acc_inc = 0.7; // m/s/s ADJUSTABLE 
long staging_link = 0;
double minimum_gap_distance=2.0; //最小停车间距 2.0m
double to_End_Dist=0.0;
int veh_ID_mod=0;
int flag_off_ramp=0;
double Lane_Change_Factor=5.0; 
double TTC_threshold=2.0;
/*==========================================================================*/

BOOL APIENTRY DllMain (HANDLE  hModule, 
                       DWORD   ul_reason_for_call, 
                       LPVOID  lpReserved)
{
  switch (ul_reason_for_call) {
      case DLL_PROCESS_ATTACH:
      case DLL_THREAD_ATTACH:
      case DLL_THREAD_DETACH:
      case DLL_PROCESS_DETACH:
         break;
  }
  return TRUE;
}

/*==========================================================================*/

DRIVERMODEL_API  int  DriverModelSetValue (long   type, 
                                           long   index1,
                                           long   index2,
                                           long   long_value,
                                           double double_value,
                                           char   *string_value)
{
  /* Sets the value of a data object of type <type>, selected by <index1> */
  /* and possibly <index2>, to <long_value>, <double_value> or            */
  /* <*string_value> (object and value selection depending on <type>).    */
  /* Return value is 1 on success, otherwise 0.                           */

  switch (type) {
    case DRIVER_DATA_PATH                   :
    case DRIVER_DATA_TIMESTEP               :
		return 1;
    case DRIVER_DATA_TIME                   :
		timestep = double_value;
		return 1;
    case DRIVER_DATA_VEH_ID                 :
		current_veh = long_value;
		return 1;
    case DRIVER_DATA_VEH_LANE               :
		current_lane = long_value;
		return 1;
    case DRIVER_DATA_VEH_ODOMETER           :
		veh_od = double_value;
		return 1;
    case DRIVER_DATA_VEH_LANE_ANGLE         :
		//lane_angle=double_value;
		return 1;
    case DRIVER_DATA_VEH_LATERAL_POSITION   :
		lateral_pos = double_value;
		return 1;
    case DRIVER_DATA_VEH_VELOCITY           :
		Spd_vehicle = double_value;
		return 1;
    case DRIVER_DATA_VEH_ACCELERATION       :
		Acc_vehicle = double_value;
		return 1;
    case DRIVER_DATA_VEH_LENGTH             :
    case DRIVER_DATA_VEH_WIDTH              :
    case DRIVER_DATA_VEH_WEIGHT             :
		return 1;
    case DRIVER_DATA_VEH_MAX_ACCELERATION   :
	  MaxAcc_vehicle = double_value;
      return 1;
    case DRIVER_DATA_VEH_TURNING_INDICATOR  :
      turning_indicator = long_value;
      return 1;
    case DRIVER_DATA_VEH_CATEGORY           :
    case DRIVER_DATA_VEH_PREFERRED_REL_LANE :
    case DRIVER_DATA_VEH_USE_PREFERRED_LANE :
      return 1;
    case DRIVER_DATA_VEH_DESIRED_VELOCITY   :
      desired_velocity = double_value;
      return 1;
    case DRIVER_DATA_VEH_X_COORDINATE       :
    case DRIVER_DATA_VEH_Y_COORDINATE       :
	  return 1;
    case DRIVER_DATA_VEH_TYPE               :
	  veh_type = long_value;
      return 1;
    case DRIVER_DATA_VEH_COLOR              :
      vehicle_color = long_value;
      return 1;
    case DRIVER_DATA_VEH_CURRENT_LINK       :
		current_link = long_value;
      return 0; /* (To avoid getting sent lots of DRIVER_DATA_VEH_NEXT_LINKS messages) */
                /* Must return 1 if these messages are to be sent from VISSIM!         */
    case DRIVER_DATA_VEH_NEXT_LINKS         :
    case DRIVER_DATA_VEH_ACTIVE_LANE_CHANGE :
    case DRIVER_DATA_VEH_REL_TARGET_LANE    :
		veh_rel_target_lane = long_value;
		return 1;
    case DRIVER_DATA_NVEH_ID                :
		AdjVehicles[index1+2][index2+2] = long_value;
		return 1;
    case DRIVER_DATA_NVEH_LANE_ANGLE        :
    case DRIVER_DATA_NVEH_LATERAL_POSITION  :
		return 1;
    case DRIVER_DATA_NVEH_DISTANCE          :
		AdjVehiclesDist[index1+2][index2+2] = double_value;
		return 1;
    case DRIVER_DATA_NVEH_REL_VELOCITY      :
		AdjVehiclesSpeed[index1+2][index2+2] = Spd_vehicle - double_value;
		return 1;
    case DRIVER_DATA_NVEH_ACCELERATION      :
		AdjVehiclesAcc[index1+2][index2+2] = double_value;
		return 1;
    case DRIVER_DATA_NVEH_LENGTH            :
		AdjVehiclesWidth[index1+2][index2+2] = double_value; // revised for VISSIM 7. 
		return 1;
    case DRIVER_DATA_NVEH_WIDTH             :
		// AdjVehiclesWidth[index1+2][index2+2] = double_value; 
		return 1;
    case DRIVER_DATA_NVEH_WEIGHT            :
    case DRIVER_DATA_NVEH_TURNING_INDICATOR :
    case DRIVER_DATA_NVEH_CATEGORY          :
		return 1;
    case DRIVER_DATA_NVEH_LANE_CHANGE       :
		AdjVehiclesLaneChange[index1+2][index2+2] = long_value;
		return 1;
    case DRIVER_DATA_NO_OF_LANES            :
		lanes_current_link = long_value;
		return 1;
    case DRIVER_DATA_LANE_WIDTH             :
    case DRIVER_DATA_LANE_END_DISTANCE      :
		to_End_Dist = double_value;
		return 1;
    case DRIVER_DATA_RADIUS                 :
    case DRIVER_DATA_MIN_RADIUS             :
    case DRIVER_DATA_DIST_TO_MIN_RADIUS     :
    case DRIVER_DATA_SLOPE                  :
    case DRIVER_DATA_SLOPE_AHEAD            :
    case DRIVER_DATA_SIGNAL_DISTANCE        :
    case DRIVER_DATA_SIGNAL_STATE           :
    case DRIVER_DATA_SIGNAL_STATE_START     :
    case DRIVER_DATA_SPEED_LIMIT_DISTANCE   :
    case DRIVER_DATA_SPEED_LIMIT_VALUE      :
      return 1;
    case DRIVER_DATA_DESIRED_ACCELERATION :
      desired_acceleration = double_value;
      return 1;
    case DRIVER_DATA_DESIRED_LANE_ANGLE :
      desired_lane_angle = double_value;
	  desired_lane_angle_tmp = double_value;
      return 1;
    case DRIVER_DATA_ACTIVE_LANE_CHANGE :
      active_lane_Change = long_value;
      return 1;
    case DRIVER_DATA_REL_TARGET_LANE :
      rel_target_lane = long_value;
      return 1;
    default :
      return 0;
  }
}

/*--------------------------------------------------------------------------*/

DRIVERMODEL_API  int  DriverModelGetValue (long   type, 
                                           long   index1,
                                           long   index2,
                                           long   *long_value,
                                           double *double_value,
                                           char   **string_value)
{
  /* Gets the value of a data object of type <type>, selected by <index1> */
  /* and possibly <index2>, and writes that value to <*double_value>,     */
  /* <*float_value> or <**string_value> (object and value selection       */
  /* depending on <type>).                                                */
  /* Return value is 1 on success, otherwise 0.                           */

  switch (type) {
    case DRIVER_DATA_STATUS :
      *long_value = 0;
      return 1;
    case DRIVER_DATA_VEH_TURNING_INDICATOR :
      *long_value = turning_indicator;
      return 1;
    case DRIVER_DATA_VEH_DESIRED_VELOCITY   :
      *double_value = desired_velocity ;
      return 1;
    case DRIVER_DATA_VEH_COLOR :
      *long_value = vehicle_color;
      return 1;
    case DRIVER_DATA_WANTS_SUGGESTION :
      *long_value = 0;
      return 1;
    case DRIVER_DATA_DESIRED_ACCELERATION :{
		
	  if (start_Control_Time[current_veh]>=0){
		 start_Control_Time[current_veh]-=1;
		 *double_value = desired_acceleration;
		 PreviousVehAcc[current_veh]=PreviousVehAcc2[current_veh];
		 PreviousVehAcc2[current_veh]=PreviousVehAcc3[current_veh];
		 PreviousVehAcc3[current_veh]=PreviousVehAcc4[current_veh];
		 PreviousVehAcc4[current_veh]=PreviousVehAcc5[current_veh];
		 PreviousVehAcc5[current_veh]=desired_acceleration;
		 return 1;
	  }else{
		 if (current_veh==VehToWatch)fout_ncacc<<"PreviousVehAcc1[current_veh]="<<PreviousVehAcc[current_veh]<<","<<PreviousVehAcc2[current_veh]<<","<<PreviousVehAcc3[current_veh]<<","<<PreviousVehAcc4[current_veh]<<","<<PreviousVehAcc5[current_veh]<<","<<endl;
		 *double_value=PreviousVehAcc[current_veh];
		 PreviousVehAcc[current_veh]=PreviousVehAcc2[current_veh];
		 PreviousVehAcc2[current_veh]=PreviousVehAcc3[current_veh];
		 PreviousVehAcc3[current_veh]=PreviousVehAcc4[current_veh];
		 PreviousVehAcc4[current_veh]=PreviousVehAcc5[current_veh];
		 PreviousVehAcc5[current_veh]=desired_acceleration;
		 return 1;
	  }
	  /*for(int i=0;i<4;i++){会报错，原因不明！！！
			PreviousVehAcc[current_veh][i]=PreviousVehAcc[current_veh][i+1];  
	  }
	  PreviousVehAcc[current_veh][4]=desired_acceleration;
	  if (current_veh==VehToWatch){
	  fout_ncacc<<"d=";
	  for(int i=0;i<5;i++){
		fout_ncacc<<PreviousVehAcc[current_veh][i]<<",";
		}
	  fout_ncacc<<",start_Control_Time="<<start_Control_Time[current_veh]<<endl;}
	  */
	}

    case DRIVER_DATA_DESIRED_LANE_ANGLE :
	  *double_value = desired_lane_angle;
      return 1;
    case DRIVER_DATA_ACTIVE_LANE_CHANGE :
      *long_value = active_lane_Change;
      return 1;
    case DRIVER_DATA_REL_TARGET_LANE :
      *long_value = rel_target_lane;
      return 1;
    case DRIVER_DATA_SIMPLE_LANECHANGE :
		  *long_value = 0;
      return 1;
    default :
      return 0;
  }
}





DRIVERMODEL_API  int  DriverModelExecuteCommand (long number)
{
  /* Executes the command <number> if that is available in the driver */
  /* module. Return value is 1 on success, otherwise 0.               */
  
  switch (number) {
    case DRIVER_COMMAND_INIT :
		now = time(0);
		dt = ctime(&now);


		fout.open("out_newdll.dat",std::ios_base::out);
		fout_ncacc.open("out_newdll_ncacc.txt",std::ios_base::out);
		fout_SAFETY.open("out_SAFETY.txt",std::ios_base::out);
		fin.open("caccconf.dat",std::ios_base::in); 
		fout_TotalSafetyResult.open("Safety_Total_101.txt",std::ios::app);
		fout<<" --------------------- "<<dt;
		fout.flush();

		if (fin) {
			fout<<"Reading parameters:"<<endl;
			getline(fin,str);
			fout<<"Platooning Acceleration :"<<desired_acc_inc<<"-->";
			desired_acc_inc = atof(str.c_str()); // m/s/s
			fout<<desired_acc_inc<<endl;

			getline(fin,str);			
			desired_dec_inc = atof(str.c_str());
			fout<<desired_dec_inc<<endl;

			getline(fin,str);			
			MaxDeceleration = atof(str.c_str());
			fout<<MaxDeceleration<<endl;

			getline(fin,str);			
			desired_dec_inc_join = atof(str.c_str()); //-1.5;
			fout<<desired_dec_inc_join<<endl;

			getline(fin,str);			
			MaxSpdForCACC = atof(str.c_str());
			fout<<MaxSpdForCACC<<endl;

			getline(fin,str);			
			MInCACCSpdBound = atof(str.c_str());
			fout<<MInCACCSpdBound<<endl;

			getline(fin,str);			
			MinFollower = atof(str.c_str()); //
			fout<<MinFollower<<endl;

			getline(fin,str);			
			MinAllowLCDist = atof(str.c_str()); // 20 m
			fout<<MinAllowLCDist<<endl;

			getline(fin,str);			
			MaxLookAheadDist = atof(str.c_str()); // 20 m
			fout<<MaxLookAheadDist<<endl;

			getline(fin,str);			
			Headway = atof(str.c_str());//0.9;
			fout<<Headway<<endl;

			getline(fin,str);			
			Headway_No = atof(str.c_str());//1.2;
			fout<<Headway_No<<endl;

			getline(fin,str);			
			staging_link = (long)atoi (str.c_str());//1.2;
			fout<<"Staging Link ID:"<<staging_link<<endl;
			
			getline(fin,str);
			VehToWatch=(long)atoi(str.c_str());
			fout<<"VehToWatch="<<VehToWatch<<endl;

		}
		else
		{
			fout<<"No Configuration Data Exsit. Use Defaults"<<endl;
		}

		fin.close();

      return 1;
    case DRIVER_COMMAND_CREATE_DRIVER :
		{
			VehStatus[current_veh] = 0;
			PreviousVehAcc[current_veh] = 0;
			PreviousVehAcc2[current_veh] = 0;
			PreviousVehAcc3[current_veh] = 0;
			PreviousVehAcc4[current_veh] = 0;
			PreviousVehAcc5[current_veh] = 0;
			start_Control_Time[current_veh]=100;
			VehTargetLane[current_veh] = 0;
			VehTag[current_veh] = 0;
			//VehInGroup[current_veh] = -1;
			VehToAdjustDecForCutIn[current_veh]=0;
			CACCLeader[current_veh] = 0;
			HowManyFollwers[current_veh] = 0;
			TTC[current_veh]=0.0;
			TET[current_veh]=0.0;
			TIT[current_veh]=0.0;
			NCJ[current_veh]=0;
			TERCRI[current_veh]=0.0;
		return 1;
		}
    case DRIVER_COMMAND_KILL_DRIVER :
		{
			if(TET[current_veh]+TET[current_veh]+NCJ[current_veh]+TERCRI[current_veh]>0) fout_SAFETY<<"ID="<<current_veh<<",type="<<veh_type<<","<<TET[current_veh]<<","<<TIT[current_veh]<<","<<TERCRI[current_veh]<<","<<NCJ[current_veh]<<endl;
			TET_Total+=TET[current_veh];
			TIT_Total+=TIT[current_veh];
			TERCRI_Total+=TERCRI[current_veh];
			NCJ_Total+=NCJ[current_veh];
			if (TET.size()<=1)fout_TotalSafetyResult<<dt<<","<<TET_Total<<","<<TIT_Total<<","<<TERCRI_Total<<","<<NCJ_Total<<endl;
			TET.erase(current_veh);
			TIT.erase(current_veh);
			TERCRI.erase(current_veh);
			NCJ.erase(current_veh);
			VehStatus.erase(current_veh);
			PreviousVehAcc.erase(current_veh);
			PreviousVehAcc2.erase(current_veh);
			PreviousVehAcc3.erase(current_veh);
			PreviousVehAcc4.erase(current_veh);
			PreviousVehAcc5.erase(current_veh);
			start_Control_Time.erase(current_veh);
			VehTargetLane.erase(current_veh);
			VehTag.erase(current_veh);
			VehToAdjustDecForCutIn.erase(current_veh);
			CACCLeader.erase(current_veh);
			HowManyFollwers.erase(current_veh);
			
			return 1;
		}

    case DRIVER_COMMAND_MOVE_DRIVER :
	{
/*>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> END OF CODE BLOCK #1>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/


/*>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> START OF CODE BLOCK #2>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
//
//

/*
                                   Primary Input/Output for CODE BLOCK #2
  
  Input Variables : double timestep, 
					int VehToWatch, 
					int JoinCase, 
					int veh_type,  
					double lateral_pos,
					double Spd_vehicle, 
					long vehicle_color, 
					int lanes_current_link, 
					int current_lane,
					int current_veh,
					double desired_dec_inc,
					int[] VehTargetLane, 
					double MInCACCSpdBound,
					int[] VehToAdjustDecForCutIn, 
					int[] VehTag,
					int[] Veh_Status,
					int[] VehToAdjustDecForCutIn
					
  Output Variables : int current_lane, 
                     int lateral_pos_ind, 
					 int rel_target_lane, 
					 int acive_lane_change, 
					 double desired_lane_angle, 
   					 int[] VehTargetLane, 
					 double MInCACCSpdBound,
					 int[] VehToAdjustDecForCutIn, 
					 int[] VehTag,
					 int[] Veh_Status,
					 int[] VehToAdjustDecForCutIn,
					 double desired_acceleration.

*/

 
      //--------- Global Setting --------------------------------------//
	  JoinCase = 0;
	  flag_off_ramp=0;//(current_veh % 20==19)?1:0;
	  flag_off_ramp=(current_link>1000)?0:flag_off_ramp; //匝道上恢复
	  if (veh_type ==101)
		 vehicle_color = RGB(255,255,255)+255*16777216;
      if (veh_type ==102)
		 vehicle_color = RGB(255,255,255)+255*16777216; // From On-Ramp 1
	  if (veh_type == 103) 
		 vehicle_color = RGB(255,255,255)+255*16777216; 

	  // set a flag for outputing vehicle information at every 1 sec. 
	  if (fmod(10.0*timestep,10.0)==0)
		  WrtFlag = false;
	  else
		  WrtFlag = false;

	  // Lane number adjustment to handle CACC vehicles on the merge/diverge areas. 

	  if (lanes_current_link > 4)
	  {
		  //fout<<current_veh<<","<<current_link<<","<<lanes_current_link<<","<<current_lane<<"-->";
		  current_lane = current_lane-(lanes_current_link-4);
		  //fout<<current_lane<<endl;		  
	  }
	  if (lanes_current_link < 4)
	  {
		  current_lane = current_lane+(4-lanes_current_link);
	  }
	  for(int i =0;i<5;i++){
		  for (int j=0;j<5;j++)
			  AdjVehiclesDist[i][j]-=AdjVehiclesWidth[i][j];
	  }
	  lateral_pos_ind = GetLateralPos(lateral_pos);

	  //if (current_veh==VehToWatch)fout_ncacc<<"VEHID="<<current_veh<<",Time="<<timestep<<",-0,active_lane_Change, current_lane,target_lane,suggestion:"<<active_lane_Change<<","<<current_lane<<","<<VehTargetLane[current_veh]<<","<<rel_target_lane<<","<<desired_lane_angle<<",lat:"<<lateral_pos<<endl;

	  //-----------------------------------------------------------------//
  
	  if (current_lane==0){
			if ((AdjVehicles[3][2]<=0)&&(AdjVehicles[3][1]<=0||-1.0*AdjVehiclesDist[3][1]>=MinAllowLCDist)&&(AdjVehicles[3][3]<=0||AdjVehiclesDist[3][3]>=MinAllowLCDist)){
			VehTargetLane[current_veh]=1;  //0:在匝道上 让刚上匝道的CAVH换道到主线上
			VehStatus[current_veh]=4;
	  }
	  }

	  
	  //专用道！！
	      	if (flag_off_ramp==0&&lanes_current_link>3&&AdjVehiclesSpeed[2][3]>10&&current_lane<4&&start_Control_Time[current_veh]<50){
				
				VehTargetLane[current_veh]=4;
				VehStatus[current_veh]=4;
				if ((current_lane!=VehTargetLane[current_veh])){
		  
					rel_target_lane =  -1;
					active_lane_Change = -1;
					desired_lane_angle = active_lane_Change * lane_angle;
				  //VehTag[current_veh]=1;
					}else{
					rel_target_lane =  0;
					active_lane_Change = 0;
					desired_lane_angle = active_lane_Change * lane_angle;
					//VehTag[current_veh]=0;
			     }
			if (lateral_pos_ind!=0){
				if (lateral_pos_ind>0)
				{
					if (current_veh==VehToWatch)fout_ncacc<<"VEHID="<<current_veh<<",Time="<<timestep<<",-3,active_lane_Change, current_lane,target_lane,suggestion:"<<active_lane_Change<<","<<current_lane<<","<<VehTargetLane[current_veh]<<","<<rel_target_lane<<","<<desired_lane_angle<<","<<endl;
					rel_target_lane =  -1;
					active_lane_Change = -1;
					desired_lane_angle = active_lane_Change * lane_angle;

				}
				else 
				{
					
					if (current_veh==VehToWatch)fout_ncacc<<"VEHID="<<current_veh<<",Time="<<timestep<<",-4,active_lane_Change, current_lane,target_lane,suggestion:"<<active_lane_Change<<","<<current_lane<<","<<VehTargetLane[current_veh]<<","<<rel_target_lane<<","<<desired_lane_angle<<","<<endl;
					rel_target_lane =  1;
					active_lane_Change = 1;
					desired_lane_angle = active_lane_Change * lane_angle;
				}
			}else{
 
			}
			
			//fout_ncacc<<"VEHID="<<current_veh<<",Time="<<flag_off_ramp<<","<<VehTargetLane[current_veh]<<","<<current_lane<<endl;
		}
	  //fout_ncacc<<"VEHID="<<current_veh<<",Time="<<flag_off_ramp<<","<<current_link<<","<<current_lane;
 
	  
//	  if ( lateral_pos_ind == 0.0){
		if ((flag_off_ramp==1) && (current_link==21||current_link==18)){
		  VehTargetLane[current_veh]=1;
		  //VehTag[current_veh]=0;
		  VehStatus[current_veh]=2;
		if ((current_lane!=VehTargetLane[current_veh])){
		  //if ((current_lane!=4)){
				  rel_target_lane =  -1;
				  active_lane_Change = -1;
				  desired_lane_angle = active_lane_Change * lane_angle;
				  //VehTag[current_veh]=1;
		}else{
				  rel_target_lane =  0;
				  active_lane_Change = 0;
				  desired_lane_angle = active_lane_Change * lane_angle;
				  //VehTag[current_veh]=0;
		} 
		if (lateral_pos_ind!=0){
			if (lateral_pos_ind>0)
			{
				if (current_veh==VehToWatch)fout_ncacc<<"VEHID="<<current_veh<<",Time="<<timestep<<",-3,active_lane_Change, current_lane,target_lane,suggestion:"<<active_lane_Change<<","<<current_lane<<","<<VehTargetLane[current_veh]<<","<<rel_target_lane<<","<<desired_lane_angle<<","<<endl;
				rel_target_lane =  -1;
				active_lane_Change = -1;
				desired_lane_angle = active_lane_Change * lane_angle;
			}
			else 
			{
					
				if (current_veh==VehToWatch)fout_ncacc<<"VEHID="<<current_veh<<",Time="<<timestep<<",-4,active_lane_Change, current_lane,target_lane,suggestion:"<<active_lane_Change<<","<<current_lane<<","<<VehTargetLane[current_veh]<<","<<rel_target_lane<<","<<desired_lane_angle<<","<<endl;
				rel_target_lane =  1;
				active_lane_Change = 1;
				desired_lane_angle = active_lane_Change * lane_angle;
			}
		}
	  }
	  //fout_ncacc<<"VEHID="<<current_veh<<",Time="<<flag_off_ramp;
	  if ((flag_off_ramp==1) && (current_link==20||current_link==18)){
		  //flag_off_ramp=1; //出匝道模式
		  VehTargetLane[current_veh]=0;
		  VehTag[current_veh]=0;
		  VehStatus[current_veh]=2;
	    if (current_lane!=VehTargetLane[current_veh]&&(lateral_pos!=0)){
				  rel_target_lane =  -1;
				  active_lane_Change = -1;
				  desired_lane_angle = active_lane_Change * lane_angle;
				  VehTag[current_veh]=0;
		}else{
				  rel_target_lane =  0;
				  active_lane_Change = 0;
				  desired_lane_angle = active_lane_Change * lane_angle;
				  VehTag[current_veh]=0;
		} 
		if (lateral_pos_ind!=0){
			if (lateral_pos_ind>0)
			{
				if (current_veh==VehToWatch)fout_ncacc<<"VEHID="<<current_veh<<",Time="<<timestep<<",-3,active_lane_Change, current_lane,target_lane,suggestion:"<<active_lane_Change<<","<<current_lane<<","<<VehTargetLane[current_veh]<<","<<rel_target_lane<<","<<desired_lane_angle<<","<<endl;
				rel_target_lane =  -1;
				active_lane_Change = -1;
				desired_lane_angle = active_lane_Change * lane_angle;
			}
			else 
			{
					
				if (current_veh==VehToWatch)fout_ncacc<<"VEHID="<<current_veh<<",Time="<<timestep<<",-4,active_lane_Change, current_lane,target_lane,suggestion:"<<active_lane_Change<<","<<current_lane<<","<<VehTargetLane[current_veh]<<","<<rel_target_lane<<","<<desired_lane_angle<<","<<endl;
				rel_target_lane =  1;
				active_lane_Change = 1;
				desired_lane_angle = active_lane_Change * lane_angle;
			}
		}
		  //fout_ncacc<<"VEHID="<<current_veh<<",Time="<<timestep<<"weaving!,current_lane:"<<current_lane<<"target_lane:"<<VehTargetLane[current_veh]<<endl;
	  }
	  //强制性变道，由run 于5.23开发
	  //judgeCurrentLaneStatus函数参数(左/右转，)
	//	if (judgeCurrentLaneStatus(1,)==1){
		//	
		//}else if{
		//	
		//}
	  
	  
	  
//	  }	
	  
	  if ((flag_off_ramp==0)&&(VehTargetLane[current_veh]==0)||(VehTargetLane[current_veh] == current_lane))
	  {
		  if (current_veh==VehToWatch)fout_ncacc<<"VEHID="<<current_veh<<",Time="<<timestep<<",-2:目标车道就是当前车道,active_lane_Change, current_lane,target_lane,suggestion:"<<active_lane_Change<<","<<current_lane<<","<<VehTargetLane[current_veh]<<","<<rel_target_lane<<","<<desired_lane_angle<<endl;
		  //if(current_lane<4&&current_link<1000)  VehTargetLane[current_veh]=4;
		  if (lateral_pos_ind !=0)
		  {
				if (lateral_pos_ind>0)
				{
					if (current_veh==VehToWatch)fout_ncacc<<"VEHID="<<current_veh<<",Time="<<timestep<<",-3，已经换道，但是方位角还没有摆正,active_lane_Change, current_lane,target_lane,suggestion:"<<active_lane_Change<<","<<current_lane<<","<<VehTargetLane[current_veh]<<","<<rel_target_lane<<","<<desired_lane_angle<<","<<endl;
					rel_target_lane =  -1;
					active_lane_Change = -1;
					desired_lane_angle = active_lane_Change * lane_angle;
					desired_acceleration = GetDesiredAcc(AdjVehiclesDist[2][3],Acc_vehicle,Spd_vehicle,AdjVehiclesAcc[2][3],AdjVehiclesSpeed[2][3],AdjVehicles[2][3], Headway,AdjVehicles[2][4],AdjVehiclesAcc[2][4],AdjVehiclesSpeed[2][4],AdjVehiclesDist[2][4]);//加速度
					//=d_a<Acc_vehicle?d_a:Acc_vehicle;
					
				}
				else 
				{
					
					if (current_veh==VehToWatch)fout_ncacc<<"VEHID="<<current_veh<<",Time="<<timestep<<",-4,active_lane_Change, current_lane,target_lane,suggestion:"<<active_lane_Change<<","<<current_lane<<","<<VehTargetLane[current_veh]<<","<<rel_target_lane<<","<<desired_lane_angle<<","<<endl;
					rel_target_lane =  1;
					active_lane_Change = 1;
					desired_lane_angle = active_lane_Change * lane_angle;
					desired_acceleration = GetDesiredAcc(AdjVehiclesDist[2][3],Acc_vehicle,Spd_vehicle,AdjVehiclesAcc[2][3],AdjVehiclesSpeed[2][3],AdjVehicles[2][3], Headway,AdjVehicles[2][4],AdjVehiclesAcc[2][4],AdjVehiclesSpeed[2][4],AdjVehiclesDist[2][4]);//加速度
				
				}
				//if (current_veh==VehToWatch)fout_ncacc<<"flag:"<<flag_off_ramp<<endl;
				//if (current_veh==VehToWatch)fout_ncacc<<"VEHID="<<current_veh<<",Time="<<timestep<<",-5,active_lane_Change, current_lane,target_lane,suggestion:"<<active_lane_Change<<","<<current_lane<<","<<VehTargetLane[current_veh]<<","<<rel_target_lane<<","<<desired_lane_angle<<","<<endl;
			//return 1;
		  }
		  else
		  {
			if (current_veh==VehToWatch)fout_ncacc<<"VEHID="<<current_veh<<",Time="<<timestep<<",-6：车道夹角为0 保持当前车道,active_lane_Change, current_lane,target_lane,suggestion:"<<active_lane_Change<<","<<current_lane<<","<<VehTargetLane[current_veh]<<","<<rel_target_lane<<","<<desired_lane_angle<<","<<endl;
			active_lane_Change = 0;
			desired_lane_angle = 0.0;
			rel_target_lane = 0;
			//if(current_lane<4&&current_link<1000)  VehTargetLane[current_veh]=4;
			//desired_acceleration = GetDesiredAcc(DistLeadingVeh,Acc_vehicle,Spd_vehicle,AdjVehiclesAcc[2][3],AdjVehiclesSpeed[2][3],AdjVehicles[2][3], Headway,AdjVehicles[2][1],AdjVehiclesAcc[2][1],AdjVehiclesSpeed[2][1]);//加速度
		  }
	  }

	  if ((flag_off_ramp==0)&&(VehStatus[current_veh]>0) && (VehTargetLane[current_veh] != current_lane) && (VehTargetLane[current_veh]>0))
	  {
		  double hw_nr=0.0;//换道时的车头时距，到隔壁车辆的前车
		  double s1=0.0;
   		  double s_nr=0.0; //临近车道的车走过的距离
		  //double t_=2.0;
		  double x;
		  double d_a=0.0;
		  double d_a1=0.0;
		  if (current_veh==VehToWatch)fout_ncacc<<"VEHID="<<current_veh<<",Time="<<timestep<<",-7：正在换道模式，目标车道不等于当前车道，目标车道>0,active_lane_Change, current_lane,target_lane,suggestion:"<<active_lane_Change<<","<<current_lane<<","<<VehTargetLane[current_veh]<<","<<rel_target_lane<<","<<desired_lane_angle<<","<<endl;
		  if (VehStatus[current_veh]<=2)
		  {
			  active_lane_Change = -1;
			  rel_target_lane =  -1;
			  desired_lane_angle = active_lane_Change * lane_angle;
			  //距离校验！校验2.5秒内的车头时距
			  
			  d_a = GetDesiredAcc(AdjVehiclesDist[1][3],Acc_vehicle,Spd_vehicle,AdjVehiclesAcc[1][3],AdjVehiclesSpeed[1][3],AdjVehicles[1][3], Headway,AdjVehicles[1][4],AdjVehiclesAcc[1][4],AdjVehiclesSpeed[1][4],AdjVehiclesDist[1][4]);//相邻车道的前车加速度
			  d_a1 = GetDesiredAcc(AdjVehiclesDist[2][3],Acc_vehicle,Spd_vehicle,AdjVehiclesAcc[2][3],AdjVehiclesSpeed[2][3],AdjVehicles[2][3], Headway,AdjVehicles[2][4],AdjVehiclesAcc[2][4],AdjVehiclesSpeed[2][4],AdjVehiclesDist[2][4]);//本车道的前车加速度
			  
			  desired_acceleration=d_a>d_a1?d_a1:d_a;
			  if (current_veh==VehToWatch)fout_ncacc<<"VEHID="<<current_veh<<",Time="<<timestep<<",-8:正在右转,active_lane_Change, current_lane,target_lane,suggestion:"<<active_lane_Change<<","<<current_lane<<","<<VehTargetLane[current_veh]<<","<<rel_target_lane<<","<<desired_lane_angle<<","<<endl;
		  }
		  if (VehStatus[current_veh]>=4)
		  {
			  active_lane_Change = 1;
			  rel_target_lane =  1;
			  desired_lane_angle = active_lane_Change * lane_angle;
			  //距离校验！校验2.5秒内的车头时距
			  d_a = GetDesiredAcc(AdjVehiclesDist[3][3],Acc_vehicle,Spd_vehicle,AdjVehiclesAcc[3][3],AdjVehiclesSpeed[3][3],AdjVehicles[3][3],Headway,AdjVehicles[3][4],AdjVehiclesAcc[3][4],AdjVehiclesSpeed[3][4],AdjVehiclesDist[3][4]);//相邻车道的前车加速度
			  d_a1 = GetDesiredAcc(AdjVehiclesDist[2][3],Acc_vehicle,Spd_vehicle,AdjVehiclesAcc[2][3],AdjVehiclesSpeed[2][3],AdjVehicles[2][3],Headway,AdjVehicles[2][4],AdjVehiclesAcc[2][4],AdjVehiclesSpeed[2][4],AdjVehiclesDist[2][4]);//本车道的前车加速度
			  if (current_veh==VehToWatch)fout_ncacc<<d_a<<","<<d_a1<<endl;
			  desired_acceleration=d_a>d_a1?d_a1:d_a;
			  if (current_veh==VehToWatch)fout_ncacc<<"VEHID="<<current_veh<<",Time="<<timestep<<",-9：正在左转,active_lane_Change, current_lane,target_lane,suggestion:"<<active_lane_Change<<","<<current_lane<<","<<VehTargetLane[current_veh]<<","<<rel_target_lane<<","<<desired_lane_angle<<","<<endl;
		  }
		  return 1;	  
	  }
	//if (current_veh==VehToWatch)fout_ncacc<<"VEHID="<<current_veh<<",Time="<<timestep<<",-1,active_lane_Change, current_lane,target_lane,suggestion:"<<active_lane_Change<<","<<current_lane<<","<<VehTargetLane[current_veh]<<",----"<<vissim_suggestion<<endl;
	//if (current_veh==VehToWatch)fout_ncacc<<VehStatus[current_veh]<<","<<endl;
 
	  if (flag_off_ramp>0 && VehTargetLane[current_veh] != current_lane )
	  {
		  fout_ncacc<<"VEHID="<<current_veh<<",Time="<<flag_off_ramp<<",-10：下匝道模式，目标车道不是当前车道,active_lane_Change, current_lane,target_lane,suggestion:"<<active_lane_Change<<","<<current_lane<<","<<VehTargetLane[current_veh]<<","<<rel_target_lane<<","<<desired_lane_angle<<","<<endl;
		  if (VehStatus[current_veh]<=2)
		  {
			  active_lane_Change = -1;
			  rel_target_lane =  -1;
			  desired_lane_angle = active_lane_Change * lane_angle;
		  }
		  //return 1;	  
	  }
	  
	  if (Spd_vehicle<MInCACCSpdBound&& VehTag[current_veh]==0) // && VehTag[current_veh]==0 )
	  {
		  //InitArrays();
		  //vissim_suggestion = 1;
		  vehicle_color = RGB(132,134,231)+255*16777216;
		  //active_lane_Change = active_lane_Change;
		  if (current_veh==VehToWatch)fout_ncacc<<"VEHID="<<current_veh<<",Time="<<timestep<<",0:车辆小于巡航速度，不处于巡航模式！,lane angle:"<<desired_lane_angle<<","<<active_lane_Change<<","<<rel_target_lane<<","<<veh_rel_target_lane<<endl;
		  //return 1;
	  
	  }

	  // The case below happens only when VISSIM provides its lane changing suggestion (vissim_suggestion == 1 && simple_lanechaging ==1). As of July 5, this is obsolute.  
	  //if (active_lane_Change!=0 && VehTag[current_veh]==0 )
	  //{
		  //InitArrays();
		  //vissim_suggestion = 1;
		  //vehicle_color = RGB(132,134,231)+255*16777216; 
		  //active_lane_Change = active_lane_Change;
		  //rel_target_lane = active_lane_Change;
          //desired_lane_angle = 0.022*active_lane_Change;
		  
		  //if (current_veh==VehToWatch)fout_ncacc<<"VEHID="<<current_veh<<",Time="<<timestep<<",0,lane angle:"<<desired_lane_angle<<","<<active_lane_Change<<","<<rel_target_lane<<","<<veh_rel_target_lane<<endl;
		  //return 1;
	  
	  //}

	  // For a CACC vehicle selected to make a room for an incoming CACC vehicle into exisiting platoon. 	
	  if (VehToAdjustDecForCutIn[current_veh]>0)
	  {
		   
		  if (current_veh==VehToWatch)fout_ncacc<<"VEHID="<<current_veh<<",Time="<<timestep<<",27:前方有车辆要cut in join 本车需要减速到距离符合要求！,lane angle:"<<desired_lane_angle<<","<<active_lane_Change<<","<<rel_target_lane<<","<<veh_rel_target_lane<<endl;
		  VehToAdjustDecForCutIn[current_veh]=0;
		desired_acceleration = GetDesiredAcc(AdjVehiclesDist[2][3],Acc_vehicle,Spd_vehicle,AdjVehiclesAcc[2][3],AdjVehiclesSpeed[2][3],AdjVehicles[2][3],Headway,AdjVehicles[2][4],AdjVehiclesAcc[2][4],AdjVehiclesSpeed[2][4],AdjVehiclesDist[2][4]);//加速度
		  InitArrays();	


		  return 1;
	  }

	  // to make a break point for debug purpose. 	

	  //----------------------------------- main logic ------------------------------------//


	  if (timestep>0.0 && ((veh_type == 101)||(veh_type == 102)))
	  {
		  ControlVehicle();
		  
	  }
	  if (timestep>1800.0&&start_Control_Time[current_veh]<=0){
		  SafetyMeassure();
	  }
	  InitArrays();	  

	  if (current_veh==VehToWatch)fout_ncacc<<"VEHID="<<current_veh<<",Time="<<timestep<<",-101：控制车辆的主逻辑结束！,active_lane_Change, current_lane,target_lane,suggestion:"<<active_lane_Change<<","<<current_lane<<","<<VehTargetLane[current_veh]<<","<<rel_target_lane<<","<<desired_lane_angle<<"lat_pos:"<<lateral_pos<<"lane_angle="<<lane_angle<<","<<endl;

      return 1;
	  }
    default :
      return 0;
	  
  }
  
}

/*>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> END OF CODE BLOCK #2>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/



/*>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> START OF CODE BLOCK #3>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
//
//
/*
                                   Primary Input/Output for CODE BLOCK #3
  
  Input Variables : double timestep, 
					int VehToWatch, 
					int veh_type,  
					double lateral_pos,
					int lateral_pos_ind, 
					int current_lane,
					int current_veh,
					int[] VehTargetLane, 
					double[][] AdjVehiclesDist,
					double[]][] AdjVehiclesAcc,
					double[][] AdjVehiclesSpeed,
					long[][] AdjVehicles. 
					
  Output Variables : long[][] AdjVehicles,
					 double[][] AdjVehiclesWidth,
					 double[][] AdjVehiclesSpeed,
			         double[][] AdjVehiclesDist[i][j],
			         double[][] AdjVehiclesAcc[i][j],
				     long[][] AdjVehiclesLaneChange[i][j], 
					 long[][] AdjVehiclesCurrentLane[i][j], 
					 long[][] AdjVehiclesTargetLane[i][j],  
					 double desired_acceleration.
*/

int InitArrays()
{
 	  for (int i=0;i<=4;i++)
	  {
		  for (int j=0;j<=4;j++)
		  {
			  AdjVehicles[i][j] = 0;
			  AdjVehiclesWidth[i][j] = 0;
			  AdjVehiclesSpeed[i][j] = 0;
			  AdjVehiclesDist[i][j] = 0;
			  AdjVehiclesAcc[i][j] = 0;
			  AdjVehiclesLaneChange[i][j] = 0;
			  AdjVehiclesCurrentLane[i][j] = 0;
			  AdjVehiclesTargetLane[i][j] = 0; 
		  }
	  }

	return 0;

}

double GetLateralPos(double latpos)
{
	if (latpos>0.2 || latpos<-0.2)
		return latpos;
	else
		return 0.0;
}
int getLaneChangeDecision(int TurningIndicator){
	double V11,V12,V13,V14,V15;//本车道五辆车车速
	//double V21,V22,V23,V24,V25;//相邻车道五辆车车速
	int n=0;//实际车辆数
	double AverageSpdThisLane,AverageSpdNearLane=0.0;
	if (TurningIndicator==+1){//向左转
		if (current_lane==4){
			return -1;  //当前已经在最左边车道，没有办法向左转
		}
		for(int i=0;i<=4;i++){
			if (AdjVehicles[2][i]==0){
				AverageSpdThisLane+=desired_velocity;//没车的时候用期望车速计算
			}else{
				AverageSpdThisLane+=AdjVehiclesSpeed[2][i];
			}
			if (AdjVehicles[3][i]==0){
				AverageSpdNearLane+=desired_velocity;//没车的时候用期望车速计算
			}else{
				AverageSpdNearLane+=AdjVehiclesSpeed[3][i];
			}
			AverageSpdThisLane/=3.0;
			AverageSpdNearLane/=3.0;
		}
		
	}else{
		if (current_lane<=1){
			return -1;  //当前已经在最右边车道，没有办法向右转
		}
		for(int i=0;i<=4;i++){
			if (AdjVehicles[2][i]==0){
				AverageSpdThisLane+=desired_velocity;//没车的时候用期望车速计算
			}else{
				AverageSpdThisLane+=AdjVehiclesSpeed[2][i];
			}
			if (AdjVehicles[1][i]==0){
				AverageSpdNearLane+=desired_velocity;//没车的时候用期望车速计算
			}else{
				AverageSpdNearLane+=AdjVehiclesSpeed[1][i];
			}
			AverageSpdThisLane/=3.0;
			AverageSpdNearLane/=3.0;
		}
	}
	
}


int SafetyMeassure(){
	//double TTC_Brake=AdjVehicles[2][3]>0?(AdjVehiclesDist[2][3]/(Spd_vehicle-AdjVehiclesSpeed[2][3])):1000000;
	double TTC_Brake=AdjVehicles[2][3]>0?(AdjVehiclesDist[2][3]/(Spd_vehicle)):1000000;
	if (Spd_vehicle-AdjVehiclesSpeed[2][3]<0) TTC_Brake=100000;
	//fout_SAFETY<<TTC_Brake<<","<<Spd_vehicle<<","<<AdjVehiclesDist[2][3]<<","<<TTC_threshold<<endl;
	if (TTC_Brake<TTC_threshold&&TTC_Brake>0){
		TET[current_veh]+=0.1;
		
		TIT[current_veh]+=(1/TTC_Brake-1/TTC_threshold)*0.1;
		
	}
	if (abs(Acc_vehicle)>5.0){
		NCJ[current_veh]+=0.1;
	}
	if (VehToWatch==current_veh)fout_ncacc<<"加速度"<<Acc_vehicle;
	double SDL=AdjVehiclesSpeed[2][3]*0.6+AdjVehiclesSpeed[2][3]*AdjVehiclesSpeed[2][3]/2/AdjVehiclesAcc[2][3]+AdjVehiclesWidth[2][3];
	double SDF=Spd_vehicle*1.5+Spd_vehicle*Spd_vehicle/2/Acc_vehicle;
	if (SDF>SDL&&AdjVehicles[2][3]>0&&Acc_vehicle<0&&AdjVehiclesAcc[2][3]<0&&AdjVehiclesAcc[2][3]<Acc_vehicle){
		TERCRI[current_veh]+=0.1;
	}
	return 1;
}





int ControlVehicle()
{
	/* Obsolute for no suggestion/no simple lane change cae
	if (active_lane_Change !=0 && VehTargetLane[current_veh] ==0)
		VehTargetLane[current_veh] = current_lane + active_lane_Change; // vissim suggests target lane. 
    */ 
		  /*
	  /* 出匝道部分代码，由Run 'S STUDIO于2019年5月2日实现
	  /*
	  */

	  

    // if a CACC vehicle runs on the lane that it wants to be or 2) just entered the network but is not willing to change lane and under CACC mode
	//if ((VehTargetLane[current_veh] == current_lane || VehTargetLane[current_veh] ==0) && lateral_pos == 0.0 && VehTag[current_veh]!=0)
    //if ((VehTargetLane[current_veh] == current_lane || VehTargetLane[current_veh] ==0) && lateral_pos_ind == 0.0 && VehTag[current_veh]!=0) 
	//{
		// initialize or keep the current maneuver. 
		//desired_lane_angle = 0.0; // do not allow to change the group once it was determined. 
	//	if (current_veh==VehToWatch) fout_ncacc<<"VEHID="<<current_veh<<",Time="<<timestep<<",2"<<endl;
	//}
	
	
	
	
	

	if (VehTargetLane[current_veh]>0 && VehTargetLane[current_veh] != current_lane)
	{
		JoinCase = 0;
		if (current_veh==VehToWatch) fout_ncacc<<"VEHID="<<current_veh<<",Time="<<timestep<<",3,current lane,target lane:"<<current_lane<<","<<VehTargetLane[current_veh]<<endl;
	}
	

	// To consider a proper leading vehicle

	tlane = 2; // look for the leading vehicle on the current lane 

	// If a CACC vehicle is willing to change lane (VehTargetLane[current_veh]>0) and its target lane is not current lane)

	/*
	if (VehTargetLane[current_veh]>0 && (VehTargetLane[current_veh] == current_lane - 1 || VehTargetLane[current_veh] == current_lane + 1))
	{
		// Check the distances of two leading vehicles on the current lane and the target lane 
		if (current_veh==VehToWatch) fout_ncacc<<"VEHID="<<current_veh<<",Time="<<timestep<<",4"<<endl;
		tlane = current_lane - 1;	
		if (current_veh==VehToWatch) fout_ncacc<<"VEHID="<<current_veh<<",Time="<<timestep<<",5"<<endl;
			// if the dist to the current lane is shorter, then set the leading vehice on the current lane
		if (AdjVehiclesDist[2][3] < AdjVehiclesDist[tlane][3])
		{	
			if (current_veh==VehToWatch) fout_ncacc<<"VEHID="<<current_veh<<",Time="<<timestep<<",6"<<endl;
			tlane = 2;
		}
			// otherwise, the leading vehicle on the target lane is the leading vehicle to consider. 
	}
  	*/

	// ----------  Scan Adjacent Vehicles --------------
	DistLeadingVeh = AdjVehiclesDist[tlane][3]; 
	DistLeadingVeh2=AdjVehiclesDist[tlane][4];
	DistLatterVeh  = AdjVehiclesDist[tlane][1];
	LeadingVeh_Acc = AdjVehiclesAcc[tlane][3];
	Lv2_Acc  = AdjVehiclesAcc[tlane][4]; 
	LeadingVeh_Spd = AdjVehiclesSpeed[tlane][3];
	Lv2_Spd  = AdjVehiclesSpeed[tlane][4];
	LeadingVeh_ID = AdjVehicles[tlane][3];
	Lv2_ID  = AdjVehicles[tlane][4];

	for (int ii=0;ii<5;ii++)
	{
		for (int jj=0;jj<5;jj++)
		{
			if (AdjVehicles[ii][jj]>0)
			{
				AdjVehiclesCurrentLane[ii][jj] = current_lane+(ii-2);
				AdjVehiclesTargetLane[ii][jj] = AdjVehiclesCurrentLane[ii][jj]+AdjVehiclesLaneChange[ii][jj];
			}
			else
			{
				AdjVehiclesCurrentLane[ii][jj] = -1;
				AdjVehiclesTargetLane[ii][jj] = -1;
			}				
		}
	}

	//if (VehTargetLane[current_veh]>0)
	//{
	//	if (AdjVehiclesTargetLane[1][3] == VehTargetLane[current_veh])
	//		return 1; // in case a GP vehicle is detected to get into a CACC's vehicles target lane, then do not get into the lane
	//}


	desired_acceleration = init_acc;
	//
	//
/*>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> END OF CODE BLOCK #3>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/


	// Determine acceleration/deceleration/lane change

/*>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> START OF CODE BLOCK #4>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
//
//

/*
                                   Primary Input/Output for CODE BLOCK #4
  
  Input Variables : double timestep,
					int VehToWatch, 
					int lateral_pos_ind, 
					int veh_type,  
					long vehicle_color, 
					int lanes_current_link, 
					int current_lane,
					int current_veh,
					itn current_link,
					int MinFollower,
					double desired_acceleration,
					int VehLengthIndicator,
					long[][] VehTargetLane,
					long[][] AdjVehicles,
					long[][] AdjVehiclesWidth,
					long[][] AdjVehiclesSpeed,
			        long[][] AdjVehiclesDist,
			        long[][] AdjVehiclesAcc,
				    long[][] AdjVehiclesLaneChange, 
					long[][] AdjVehiclesCurrentLane, 
					long[][] AdjVehiclesTargetLane.  
					
  Output Variables : long[][] AdjVehicles,
					 long[][] AdjVehiclesWidth,
					 long[][] AdjVehiclesSpeed,
			         long[][] AdjVehiclesDist[i][j],
			         long[][] AdjVehiclesAcc[i][j],
				     long[][] AdjVehiclesLaneChange, 
					 long[][] AdjVehiclesCurrentLane, 
					 long[][] AdjVehiclesTargetLane, 
					 long[] CACCLeader,
					 int DistLeadingVeh,
					 double Acc_vehicle,
					 double Spd_vehicle,
					 double LeadingVeh_Acc,
					 double LeadingVeh_Spd,
					 int LeadingVeh_ID, 
					 double Headway,
					 double desired_acceleration.
*/

	// if a CACC vehicle runs on the lane that it wants to be or 2) just entered the network but is not willing to change lane
	if ((VehTargetLane[current_veh] == current_lane || VehTargetLane[current_veh] ==0) && lateral_pos_ind == 0.0)
	{
		// Check the leading vehicle whether it exists or not
		// if existing
		if (current_veh==VehToWatch) fout_ncacc<<"VEHID="<<current_veh<<",Time="<<timestep<<",7"<<endl;
		if (LeadingVeh_ID>0 && DistLeadingVeh<MaxLookAheadDist)
		{	
			//Check whether it is an equipped vehicle or not. 

			if (AdjVehiclesWidth[2][3]==VehLengthIndicator)// if equipped,  // Note Nov. 11 2014: if (AdjVehiclesWidth[2][3]!=VehLengthIndicator) for VISSIM 5.4
			{//i是左右 j 是前面，i=2 j=3 正前方
				if (current_veh==VehToWatch) fout_ncacc<<"VEHID="<<current_veh<<",Time="<<timestep<<",8"<<endl;
				// Get the optimal accel/decel to keep the headway
				desired_acceleration = GetDesiredAcc(DistLeadingVeh,Acc_vehicle,Spd_vehicle,LeadingVeh_Acc,LeadingVeh_Spd,LeadingVeh_ID, Headway,Lv2_ID,Lv2_Acc,Lv2_Spd,DistLeadingVeh2);
				//if (Spd_vehicle>0.8*desired_velocity)
					VehTag[current_veh]=1;
				vehicle_color = RGB(255,0,255)+255*16777216; //处于巡航状态的车辆：绿色 ，表示前方是CACC
				CACCLeader[current_veh] = CACCLeader[LeadingVeh_ID];
				if (WrtFlag)
				{
					fout<<timestep<<","<<veh_od<<",1,"<<current_veh<<","<<LeadingVeh_ID<<endl;

					//VehInGroup[current_veh] = 1;
				}
			}
			else // if not equipped, 
			{   // no CACC vehicle ahead; it works as a ACC with 1.2 seconds of headway 
				if (current_veh==VehToWatch) fout_ncacc<<"VEHID="<<current_veh<<",Time="<<timestep<<",9：前方车辆不是CACC，以1.2车头时距为目标巡航"<<endl;
				desired_acceleration = GetDesiredAcc(DistLeadingVeh,Acc_vehicle,Spd_vehicle,LeadingVeh_Acc,LeadingVeh_Spd,LeadingVeh_ID, Headway_No,Lv2_ID,Lv2_Acc,Lv2_Spd,DistLeadingVeh2);		


				CACCLeader[current_veh] = current_veh;
				int followers = 0;
				for( map<long,int>::iterator it=CACCLeader.begin(); it!=CACCLeader.end(); ++it)
				{
			       if ((*it).second == current_veh)
					   followers++;
				}

				// tag it as non-CACC mode, which means it is allowed to search for adjacent CACC group

				//fout_ncacc<<"VEHID="<<current_veh<<",Time="<<timestep<<","<<veh_od<<",0,"<<current_veh<<","<<followers<<endl;
				if (followers <=MinFollower){							
					VehTag[current_veh]=0; 
					if (current_veh==VehToWatch) fout_ncacc<<"VEHID="<<current_veh<<",Time="<<timestep<<",10：巡航模式关闭,小于最小跟车车辆数VehTag:"<<VehTag[current_veh]<<endl;
				}
				else if (followers > MaxFollower)
				{
					VehTag[current_veh]=0; 
					if (current_veh==VehToWatch) fout_ncacc<<"VEHID="<<current_veh<<",Time="<<timestep<<",10：巡航模式关闭，超过最大跟车车辆数VehTag:"<<VehTag[current_veh]<<endl;
				}else{
					VehTag[current_veh]=1;
					if (current_veh==VehToWatch) fout_ncacc<<"VEHID="<<current_veh<<",Time="<<timestep<<",10：巡航模式打开,VehTag:"<<VehTag[current_veh]<<endl;
				}
				if (WrtFlag)
					fout<<timestep<<","<<veh_od<<",0,"<<current_veh<<","<<LeadingVeh_ID<<endl;

				vehicle_color = RGB(255,0,0)+255*16777216; ; 

			}
		}
		else // if no vehicle ahead
		{
			//maintain desired speed
			if (Spd_vehicle <= desired_velocity)
				desired_acceleration = GetDesiredAcc(DistLeadingVeh,Acc_vehicle,Spd_vehicle,LeadingVeh_Acc,LeadingVeh_Spd,LeadingVeh_ID, Headway,Lv2_ID,Lv2_Acc,Lv2_Spd,DistLeadingVeh2);	
			else
				desired_acceleration = 0.0;	
					
			if (current_veh==VehToWatch) fout_ncacc<<"VEHID="<<current_veh<<",Time="<<timestep<<",11:没有车辆在前方，维持期望速度,desiredACC"<<desired_acceleration <<endl;

			// tag it as non-CACC mode, which means it is allowed to search for adjacent CACC group
			VehTag[current_veh]=0;
			CACCLeader[current_veh] = current_veh;

			int followers = 0;
			//HowManyFollwers[current_veh]=0;
			for( map<long,int>::iterator it=CACCLeader.begin(); it!=CACCLeader.end(); ++it)
			{
				//HowManyFollwers[current_veh]++;
			   if ((*it).second == current_veh)
				   followers++;
			}
			//fout_ncacc<<"VEHID="<<current_veh<<",Time="<<timestep<<",HowManyFollwers:"<<HowManyFollwers[current_veh]<<endl;
			//fout_ncacc<<"VEHID="<<current_veh<<",Time="<<timestep<<",followers:"<<followers<<endl;
			//
			// tag it as non-CACC mode, which means it is allowed to search for adjacent CACC group

			//fout_ncacc<<"VEHID="<<current_veh<<",Time="<<timestep<<","<<veh_od<<",-1,"<<current_veh<<","<<followers<<endl; 
			vehicle_color = RGB(0,255,127)+255*16777216; ; 
			if (followers <=MinFollower){							
					VehTag[current_veh]=0; 
				if (current_veh==VehToWatch) fout_ncacc<<"VEHID="<<current_veh<<",Time="<<timestep<<",12：非巡航模式 换道模式关闭,小于最小跟车车辆数VehTag:"<<VehTag[current_veh]<<endl;
			}
			else if (followers > MaxFollower)
			{
				VehTag[current_veh]=0; 
				if (current_veh==VehToWatch) fout_ncacc<<"VEHID="<<current_veh<<",Time="<<timestep<<",12：非巡航模式，超过最大跟车车辆数VehTag:"<<VehTag[current_veh]<<endl;
			}else{
				VehTag[current_veh]=1;
				if (current_veh==VehToWatch) fout_ncacc<<"VEHID="<<current_veh<<",Time="<<timestep<<",12：非巡航模式,VehTag:"<<VehTag[current_veh]<<endl;
			}
			if (current_veh==VehToWatch) fout_ncacc<<"VEHID="<<current_veh<<",Time="<<timestep<<",12：非巡航模式，可以搜索周围车,换道模式打开状态：VehTag:"<<VehTag[current_veh]<<endl;

			if (WrtFlag)
			{
				fout<<timestep<<","<<veh_od<<",-1,"<<current_veh<<","<<LeadingVeh_ID<<endl;
				//VehInGroup[current_veh] = 0;
			}
		}
	}
	

	// If a CACC vehicle is willing to change lane (VehTargetLane[current_veh]>0) and its target lane is not current lane
	if ((VehTargetLane[current_veh] >0 && VehTargetLane[current_veh] != current_lane)||((VehTargetLane[current_veh]==0) && (flag_off_ramp>0)))
	{
		// if no lane change is in progress but a leading vehicle exisits on the target lane
		if (lateral_pos_ind == 0.0 && LeadingVeh_ID>0)
		{
			if (AdjVehiclesWidth[tlane][3]==VehLengthIndicator) // for VISSIM 5.4 : (AdjVehiclesWidth[tlane][3]!=VehLengthIndicator) 
			{
				// logically, it should not happen as the CACC vehicle in the middle of group is not allowed to change lane
				desired_acceleration = GetDesiredAcc(DistLeadingVeh,Acc_vehicle,Spd_vehicle,LeadingVeh_Acc,LeadingVeh_Spd,LeadingVeh_ID, Headway,Lv2_ID,Lv2_Acc,Lv2_Spd,DistLeadingVeh2);		
				if (current_veh==VehToWatch) fout_ncacc<<"VEHID="<<current_veh<<",Time="<<timestep<<",13：理论上是不会发生的，因为在队列中的CACC是不允许换道的"<<endl;
			}
			else
			{
				desired_acceleration = GetDesiredAcc(DistLeadingVeh,Acc_vehicle,Spd_vehicle,LeadingVeh_Acc,LeadingVeh_Spd,LeadingVeh_ID, Headway_No,Lv2_ID,Lv2_Acc,Lv2_Spd,DistLeadingVeh2);	
				//VehTag[current_veh] = 0;
				if (current_veh==VehToWatch) fout_ncacc<<"VEHID="<<current_veh<<",Time="<<timestep<<",14：前方车辆是普通车，用普通的车头时距计算"<<endl;
			}

		}
		// if lane change is in progress but a leading vehicle exisits on the target lane
		if (lateral_pos_ind != 0.0 && tlane !=2 && LeadingVeh_ID>0)
		{
			// if the leading vehile is CACC
			if (AdjVehiclesWidth[tlane][3]==VehLengthIndicator) //// for VISSIM 5.4 : (AdjVehiclesWidth[tlane][3]!=VehLengthIndicator)
			{
				if (current_veh==VehToWatch) fout_ncacc<<"VEHID="<<current_veh<<",Time="<<timestep<<",15"<<endl;
				desired_acceleration = GetDesiredAcc(DistLeadingVeh,Acc_vehicle,Spd_vehicle,LeadingVeh_Acc,LeadingVeh_Spd,LeadingVeh_ID, Headway,Lv2_ID,Lv2_Acc,Lv2_Spd,DistLeadingVeh2);		
			}
			else
			{
				if (current_veh==VehToWatch) fout_ncacc<<"VEHID="<<current_veh<<",Time="<<timestep<<",16"<<endl;
				desired_acceleration = GetDesiredAcc(DistLeadingVeh,Acc_vehicle,Spd_vehicle,LeadingVeh_Acc,LeadingVeh_Spd,LeadingVeh_ID, Headway_No,Lv2_ID,Lv2_Acc,Lv2_Spd,DistLeadingVeh2);		
			}
		}
	}

	// If vehicle is in the CACC group, do not allow lane change. 
	if (VehTag[current_veh]==1 )
	{//车辆在巡航中打开的情况
		if (current_veh==VehToWatch) fout_ncacc<<"VEHID="<<current_veh<<",Time="<<timestep<<",17:在队列中不能换道。"<<endl;

		vehicle_color = RGB(255,255,0)+255*16777216; ; 
		//desired_lane_angle = 0.0;
		//vissim_suggestion = 0;
		//simple_lanechange = 0;
		if(AdjVehiclesLaneChange[1][3]>0&&AdjVehiclesTargetLane[1][3]==current_lane){
			double d_a=0.0;
			double d_a1=0.0;
			d_a = GetDesiredAcc(AdjVehiclesDist[1][3],Acc_vehicle,Spd_vehicle,AdjVehiclesAcc[1][3],AdjVehiclesSpeed[1][3],AdjVehicles[1][3], Headway,Lv2_ID,Lv2_Acc,Lv2_Spd,AdjVehiclesDist[1][4]);	
			d_a1=GetDesiredAcc(AdjVehiclesDist[2][3],Acc_vehicle,Spd_vehicle,AdjVehiclesAcc[2][3],AdjVehiclesSpeed[2][3],AdjVehicles[2][3], Headway,Lv2_ID,Lv2_Acc,Lv2_Spd,AdjVehiclesDist[2][4]);
			desired_acceleration=d_a>d_a1?d_a1:d_a;
			if (current_veh==VehToWatch) fout_ncacc<<"VEHID="<<current_veh<<",Time="<<timestep<<",17.5:有车辆从右边加入！而且距离比到当前头车的距离要近！重新计算加速度="<<desired_acceleration<<endl;
			return 1; 
		}
		if(AdjVehiclesLaneChange[3][3]<0&&AdjVehiclesTargetLane[3][3]==current_lane){
			double d_a=0.0;
			double d_a1=0.0;
			d_a = GetDesiredAcc(AdjVehiclesDist[3][3],Acc_vehicle,Spd_vehicle,AdjVehiclesAcc[3][3],AdjVehiclesSpeed[3][3],AdjVehicles[3][3], Headway,Lv2_ID,Lv2_Acc,Lv2_Spd,AdjVehiclesDist[3][4]);
			d_a1=GetDesiredAcc(AdjVehiclesDist[2][3],Acc_vehicle,Spd_vehicle,AdjVehiclesAcc[2][3],AdjVehiclesSpeed[2][3],AdjVehicles[2][3], Headway,Lv2_ID,Lv2_Acc,Lv2_Spd,AdjVehiclesDist[2][4]);	desired_acceleration=d_a>d_a1?d_a1:d_a;				
			if (current_veh==VehToWatch) fout_ncacc<<"VEHID="<<current_veh<<",Time="<<timestep<<",17.5:有车辆从左边加入！而且距离比到当前头车的距离要近！重新计算加速度="<<desired_acceleration<<endl;
			return 1; 
		}
		//return 1; 
		
	}

	//Check: if lane change decision is given by vissim, ignore it. 

	// If a CACC vehicle is not in the CACC group or the most front CACC, and not in the lane change mode, look for adjacent CACC group
	//if(active_lane_Change ==0 && lateral_pos==0.0)

/*>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> END OF CODE BLOCK #4>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/


/*>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> START OF CODE BLOCK #5>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
//
//

/*
                                   Primary Input/Output for CODE BLOCK #5
  
  Input Variables : double timestep,
					int active_lane_Change,  
					long[] VehTargetLane,
					int VehToWatch, 
					int lateral_pos_ind, 
					int MinAllowLCDist,
					int join_case, 
					double DistTargetLeadingVeh
					long[][] AdjVehicles,
			        long[][] AdjVehiclesDist.  
					
  Output Variables : int join_case,
  				     int decelFlag1,
				     int decelFlag0. 
  */

  	if((active_lane_Change ==0 || VehTargetLane[current_veh] == current_lane)&&current_lane!=0 && lateral_pos_ind==0.0&& VehTag[current_veh]==1&& desired_velocity-Spd_vehicle>Lane_Change_Factor)// )
	{//处于巡航状态，但是速度太低，选择性换道
		//VehTag[current_veh]=0;//解除巡航状态
		double MinDist = 10000.0;
		double dx1 = 0.0;
		double dx2 = 0.0;
		int decelFlag1 = 0;
		int decelFlag0 = 0;
		double ave_velocity_nr=0.0;
		double ave_velocity_hr=0.0;
		if (current_veh==VehToWatch)
			fout_ncacc<<"VEHID="<<current_veh<<",Time="<<timestep<<",当前车道车速太低，选择性换道"<<endl;
		// Cut-in Join
		
		for(int i=1;i<=3;i=i+2) // it should start from 1 to get it activated
		{
			dx1 = 0.0;
			dx2 = 0.0;
			int veh_Count=0;
			ave_velocity_hr+=AdjVehicles[i][3]?AdjVehiclesSpeed[i][3]:desired_velocity;
			ave_velocity_hr+=AdjVehicles[i][1]?AdjVehiclesSpeed[i][1]:desired_velocity;
			ave_velocity_nr/=2.0;
			
			// if a CACC group is identified on either right or left lane and both leading and following vehicls on the adjacent lane are all CACCs
			if (JoinCase==0 &&((decelFlag1 + decelFlag0)== 0) && AdjVehicles[i][3]>0 && AdjVehiclesWidth[i][3]==VehLengthIndicator && AdjVehicles[i][1]>0 && AdjVehiclesWidth[i][1]==VehLengthIndicator&&ave_velocity_nr-Spd_vehicle>Lane_Change_Factor) // 相邻车道车速比本车道高21km/h
			{	
				DistTargetLeadingVeh = AdjVehiclesDist[i][3];
				//if (current_veh==VehToWatch) 
				if (current_veh==VehToWatch)fout_ncacc<<"选择性换道：临近车道两辆车都是CACC,CUT-IN-JOIN，"<<((i==1)?"往右":"往左")<<",当前该队列车辆数："<<endl;
				// measure distances from those leading/following vehicles 
				dx1 = AdjVehiclesDist[i][3];//i 是左右，j 是前后
				dx2 = -1.0*AdjVehiclesDist[i][1];
				if (current_veh==VehToWatch) fout_ncacc<<"到目标头车的距离="<<DistTargetLeadingVeh<<",到前车距离="<<dx1<<",到后车距离="<<dx2<<endl;
				if (current_veh==VehToWatch) fout_ncacc<<"AdjVehicles:前车编号："<<AdjVehicles[i][3]<<",后车编号:"<<AdjVehicles[i][1];
				// if the gap between the following vehicle is not enought, have it decelerate until the gap is > MinAllowLCDist+10
				if (dx2<MinAllowLCDist)
				{
					//VehToAdjustDecForCutIn[AdjVehicles[i][1]]=1;
					decelFlag1 = AdjVehicles[i][1];
				}  

				// if the gaps for both leading and following vehicles are enough, have the subject vehicle change the lane 
				if (dx1>MinAllowLCDist && dx2>MinAllowLCDist )
				{
					JoinCase = i+1; //=2或4
					MinDist = DistTargetLeadingVeh; 
				}

				if (dx1>0 && dx1<=MinAllowLCDist && Spd_vehicle > 0.8*desired_velocity)
				{
					if (current_veh==VehToWatch) fout_ncacc<<"到后车的距离过近,且本车处于巡航状态"<<endl;
					decelFlag0 = 1;
				}
			} 

		}
		//back join
		if (((decelFlag1 + decelFlag0)== 0) && JoinCase==0)
		{
			if (current_veh==VehToWatch)
				fout_ncacc<<"VEHID="<<current_veh<<",Time="<<timestep<<",20：选择性换道！"<<endl;
			
			for(int i=1;i<=3;i=i+2)
			{
				dx1 = 0.0;
				dx2 = 0.0;
				ave_velocity_nr=AdjVehicles[i][3]?AdjVehiclesSpeed[i][3]:desired_velocity;
				if (current_veh==VehToWatch) fout_ncacc<<"VEHID="<<current_veh<<",Time="<<timestep<<",20：尾部加入,"<<AdjVehicles[i][3]<<","<<AdjVehiclesWidth[i][3]<<","<<AdjVehiclesWidth[i][1]<<","<<AdjVehiclesWidth[i][1]<<endl;
				if(AdjVehicles[i][3]>0 && AdjVehiclesWidth[i][3]==VehLengthIndicator && (AdjVehiclesWidth[i][1]!=VehLengthIndicator || AdjVehicles[i][1]==0)&&ave_velocity_nr-Spd_vehicle>Lane_Change_Factor) 
				{
					//JoinCase = i+1;
					DistTargetLeadingVeh = AdjVehiclesDist[i][3];
					if (i<2)
					{
						dx1 = AdjVehiclesDist[1][3];
						dx2 = -1.0*AdjVehiclesDist[1][1];
						if (dx2<MinAllowLCDist)
						{
							//VehToAdjustDecForCutIn[AdjVehicles[i][1]]=1;
							decelFlag1 = AdjVehicles[i][1];
						} 
						if (current_veh==VehToWatch) 
							fout_ncacc<<"VEHID="<<current_veh<<",Time="<<timestep<<",21:选择性Back JOIN-往右边"<<endl;
					}
					if (i>2)
					{
						dx1 = AdjVehiclesDist[3][3];
						dx2 = -1.0*AdjVehiclesDist[3][1];
						if (dx2<MinAllowLCDist)
						{
							//VehToAdjustDecForCutIn[AdjVehicles[i][1]]=1;
							decelFlag1 = AdjVehicles[i][1];
						} 
						if (current_veh==VehToWatch) 
							fout_ncacc<<"VEHID="<<current_veh<<",Time="<<timestep<<",23：选择性Back-Join：往左边"<<endl;
					}
				 }
			}
		}
			
		if (decelFlag1 >0) 
			VehToAdjustDecForCutIn[decelFlag1]=1;

		 if (current_veh==VehToWatch)fout_ncacc<<"VEHID="<<current_veh<<",Time="<<timestep<<",251, JoinCase:选择性换道"<<JoinCase<<endl;

		switch(JoinCase){
		
		if (current_veh==VehToWatch) fout_ncacc<<"VEHID="<<current_veh<<",Time="<<timestep<<",26, Join:"<<JoinCase<<endl;
		//case 0:
		//	active_lane_Change = 0;
		//	break;
		case 1:
		if ((lateral_pos!=0)){
			active_lane_Change = -1;
			rel_target_lane =  - 1;
			desired_lane_angle = active_lane_Change * lane_angle;
			VehStatus[current_veh] = JoinCase;
			//VehTargetLane[current_veh] = current_lane - 2;
			VehTargetLane[current_veh] = current_lane - 1;
			if (current_veh==VehToWatch)fout_ncacc<<"VEHID="<<current_veh<<",Time="<<timestep<<",currlane="<<current_lane<<",26,JoinCase:"<<JoinCase<<endl;
		}else{
			active_lane_Change = 0;
			VehStatus[current_veh] = JoinCase;
			VehTargetLane[current_veh]=current_lane-1;
			}
			break;
		case 2:
		if ((lateral_pos!=0)){
			active_lane_Change = -1;
			rel_target_lane =  -1;
			desired_lane_angle = active_lane_Change * lane_angle;
			VehStatus[current_veh] = JoinCase;
			//VehTargetLane[current_veh] = current_lane - 2;
			VehTargetLane[current_veh] = current_lane - 1;
			if (current_veh==VehToWatch)fout_ncacc<<"VEHID="<<current_veh<<",Time="<<timestep<<",currlane="<<current_lane<<",JoinCase:"<<JoinCase<<endl;
		}else{
			active_lane_Change = 0;
			VehStatus[current_veh] = JoinCase;
			VehTargetLane[current_veh]=current_lane-1;
		}
			break;
		case 3:
			active_lane_Change = 0;
			VehStatus[current_veh] = JoinCase;
			VehTargetLane[current_veh] = current_lane;
			//if(Spd_vehicle>0.8*desired_velocity)
				VehTag[current_veh]=1;
			break;
		case 4:
			if ((lateral_pos!=0)){//修正此处可能会报错的情况
				active_lane_Change =+1;
				rel_target_lane =+1; //-1是往右！
				desired_lane_angle = active_lane_Change * lane_angle;
				VehStatus[current_veh] = JoinCase;
				VehTargetLane[current_veh] = current_lane + 1;
				if (current_veh==VehToWatch)fout_ncacc<<"VEHID="<<current_veh<<",Time="<<timestep<<",currlane="<<current_lane<<",JoinCase:"<<JoinCase<<endl;
			}else{
				active_lane_Change = 0;
				VehStatus[current_veh] = JoinCase;
				VehTargetLane[current_veh]=current_lane+1;
			}
			break;
		case 5:
			if ((lateral_pos!=0)){
				active_lane_Change =+1;
				rel_target_lane =+1; 
				desired_lane_angle = active_lane_Change * lane_angle;
				VehStatus[current_veh] = JoinCase;
				VehTargetLane[current_veh] = current_lane + 1;
				if (current_veh==VehToWatch)fout_ncacc<<"VEHID="<<current_veh<<",Time="<<timestep<<",currlane="<<current_lane<<",JoinCase:"<<JoinCase<<endl;
			}else{
				active_lane_Change = 0;
				VehStatus[current_veh] = JoinCase;
				VehTargetLane[current_veh]=current_lane+1;
			}
			break;
		default:
			break;
		}			
	}
	if((active_lane_Change ==0 || VehTargetLane[current_veh] == current_lane || VehTargetLane[current_veh] ==0) && lateral_pos_ind==0.0&& VehTag[current_veh]==0)// )	
	{//处于非巡航状态！！
		double MinDist = 10000.0;
		double dx1 = 0.0;
		double dx2 = 0.0;
		int decelFlag1 = 0;
		int decelFlag0 = 0;

		if (current_veh==VehToWatch) fout_ncacc<<"VEHID="<<current_veh<<",Time="<<timestep<<",19"<<endl;
		// Cut-in Join
		 
		for(int i=1;i<=3;i=i+2) // it should start from 1 to get it activated
		{
			dx1 = 0.0;
			dx2 = 0.0;
			// if a CACC group is identified on either right or left lane and both leading and following vehicls on the adjacent lane are all CACCs
			if (JoinCase==0 &&((decelFlag1 + decelFlag0)== 0) && AdjVehicles[i][3]>0 && AdjVehiclesWidth[i][3]==VehLengthIndicator && AdjVehicles[i][1]>0 && AdjVehiclesWidth[i][1]==VehLengthIndicator) // for VISSIM 5.4 : (AdjVehiclesWidth[tlane][3]!=VehLengthIndicator)
			{
				DistTargetLeadingVeh = AdjVehiclesDist[i][3];
				if (current_veh==VehToWatch) fout_ncacc<<"临近车道两辆车都是CACC,CUT-IN-JOIN，"<<((i==1)?"往右":"往左")<<",当前该队列车辆数："<<endl;
				// measure distances from those leading/following vehicles 
				dx1 = AdjVehiclesDist[i][3];//i 是左右，j 是前后
				dx2 = -1.0*AdjVehiclesDist[i][1];
				if (current_veh==VehToWatch) fout_ncacc<<"到目标头车的距离="<<DistTargetLeadingVeh<<",到前车距离="<<dx1<<",到后车距离="<<dx2<<endl;
				if (current_veh==VehToWatch) fout_ncacc<<"AdjVehicles:前车编号："<<AdjVehicles[i][3]<<",后车编号:"<<AdjVehicles[i][1];
				// if the gap between the following vehicle is not enought, have it decelerate until the gap is > MinAllowLCDist+10
				if (dx2<MinAllowLCDist)
				{
					//VehToAdjustDecForCutIn[AdjVehicles[i][1]]=1;
					decelFlag1 = AdjVehicles[i][1];
				}  

				// if the gaps for both leading and following vehicles are enough, have the subject vehicle change the lane 
				if (dx1>MinAllowLCDist && dx2>MinAllowLCDist )
				{
					JoinCase = i+1; //=2或4
					MinDist = DistTargetLeadingVeh; 
				}

				if (dx1>0 && dx1<=MinAllowLCDist && Spd_vehicle > 0.8*desired_velocity)
				{
					if (current_veh==VehToWatch) fout_ncacc<<"到后车的距离过近,且本车处于巡航状态"<<endl;
					decelFlag0 = 1;
				}
			} 

		}

/*>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> END OF CODE BLOCK #5>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/


/*>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> START OF CODE BLOCK #6>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
//
//
/*
                                   Primary Input/Output for CODE BLOCK #6
  
  Input Variables : int VehToWatch, 
					int lateral_pos_ind, 
					int veh_type,  
					int lanes_current_link, 
					int current_lane,
					int current_veh,
					itn current_link,
					double lane_angle,
				    int join_case,
  				    int decelFlag1,
				    int decelFlag0,
					long[][] AdjVehicles,
					long[][] AdjVehiclesWidth,
			        long[][] AdjVehiclesDist[i][j],
			        long[][] AdjVehiclesAcc[i][j]. 

					
  Output Variables : long[][] VehTargetLane,
					 int active_lane_Change,  
					 int rel_target_lane, 
					 double desired_lane_angle,
					 int[] VehStatus,
					 int[] VehTargetLan,
					 double desired_acceleration.
*/
		// Back Join 
		if (((decelFlag1 + decelFlag0)== 0) && JoinCase==0)
		{
			if (current_veh==VehToWatch) fout_ncacc<<"VEHID="<<current_veh<<",Time="<<timestep<<",20"<<endl;

			for(int i=0;i<=4;i++)
			{
				dx1 = 0.0;
				dx2 = 0.0;
				if (current_veh==VehToWatch) fout_ncacc<<"VEHID="<<current_veh<<",Time="<<timestep<<",20：尾部加入,"<<AdjVehicles[i][3]<<","<<AdjVehiclesWidth[i][3]<<","<<AdjVehiclesWidth[i][1]<<","<<AdjVehiclesWidth[i][1]<<endl;
				if(AdjVehicles[i][3]>0 && AdjVehiclesWidth[i][3]==VehLengthIndicator && (AdjVehiclesWidth[i][1]!=VehLengthIndicator || AdjVehiclesWidth[i][1]==0)  ) // for VISSIM 5.4 : (AdjVehiclesWidth[tlane][3]!=VehLengthIndicator)
				{
					//JoinCase = i+1;
					DistTargetLeadingVeh = AdjVehiclesDist[i][3];

					if (i<2)
					{
						dx1 = AdjVehiclesDist[1][3];
						dx2 = -1.0*AdjVehiclesDist[1][1];
						if (dx2<MinAllowLCDist)
						{
							//VehToAdjustDecForCutIn[AdjVehicles[i][1]]=1;
							decelFlag1 = AdjVehicles[i][1];
						} 
						if (current_veh==VehToWatch) fout_ncacc<<"VEHID="<<current_veh<<",Time="<<timestep<<",21:Back JOIN-从右边"<<endl;
					}
					if (i==2)
					{
						dx1 = MinAllowLCDist+1;
						dx2 = MinAllowLCDist+1;
						if (current_veh==VehToWatch) fout_ncacc<<"VEHID="<<current_veh<<",Time="<<timestep<<",22：Back-Join：本车道"<<endl;
					}

					if (i>2)
					{
						dx1 = AdjVehiclesDist[3][3];
						dx2 = -1.0*AdjVehiclesDist[3][1];
						if (dx2<MinAllowLCDist)
						{
							//VehToAdjustDecForCutIn[AdjVehicles[i][1]]=1;
							decelFlag1 = AdjVehicles[i][1];
						} 
						if (current_veh==VehToWatch) fout_ncacc<<"VEHID="<<current_veh<<",Time="<<timestep<<",23：Back-Join：从左边"<<endl;
					}
					if (dx1>MinAllowLCDist && dx2>MinAllowLCDist && DistTargetLeadingVeh < MinDist )
					{
						JoinCase = i+1;
						MinDist = DistTargetLeadingVeh; // have it select the nearest CACC vehicle. 
						if (current_veh==VehToWatch) fout_ncacc<<"VEHID="<<current_veh<<",Time="<<timestep<<"选择距离最近的头车"<<endl;
					}
				 }
			}

			if (JoinCase >0)
			{
				//if (decelFlag1 >0) 
				//VehToAdjustDecForCutIn[decelFlag1]=1;
				//decelFlag0 = 0;
				//decelFlag1 = 0;
				//if (current_veh==VehToWatch)fout_ncacc<<"VEHID="<<current_veh<<",Time="<<timestep<<",25: 不是cut-in join就不会发生这种情况"<<endl;
				
				//if (dx1>0 && dx1<=MinAllowLCDist+10 && Spd_vehicle > 0.8*desired_velocity)
				//{
				//	desired_acceleration = desired_dec_inc;
				//}
			}
		}


		if (decelFlag1 >0) 
			VehToAdjustDecForCutIn[decelFlag1]=1;
		//if (decelFlag0 >0)
		//	desired_acceleration = desired_dec_inc;

		 if (current_veh==VehToWatch)fout_ncacc<<"VEHID="<<current_veh<<",Time="<<timestep<<",251, JoinCase:"<<JoinCase<<endl;

		//if (current_link == 6)
			//JoinCase = 0;

		switch(JoinCase){
		
			if (current_veh==VehToWatch) fout_ncacc<<"VEHID="<<current_veh<<",Time="<<timestep<<",26, Join:"<<JoinCase<<endl;
		//case 0:
		//	active_lane_Change = 0;
		//	break;
		case 1:
		if ((lateral_pos!=0)){
			active_lane_Change = -1;
			rel_target_lane =  - 1;
			desired_lane_angle = active_lane_Change * lane_angle;
			VehStatus[current_veh] = JoinCase;
			//VehTargetLane[current_veh] = current_lane - 2;
			VehTargetLane[current_veh] = current_lane - 1;
			if (current_veh==VehToWatch)fout_ncacc<<"VEHID="<<current_veh<<",Time="<<timestep<<",currlane="<<current_lane<<",26,JoinCase:"<<JoinCase<<endl;
		}else{
			active_lane_Change = 0;
			VehStatus[current_veh] = JoinCase;
			VehTargetLane[current_veh]=current_lane-1;
			}
			break;
		case 2:
		if ((lateral_pos!=0)){
			active_lane_Change = -1;
			rel_target_lane =  -1;
			desired_lane_angle = active_lane_Change * lane_angle;
			VehStatus[current_veh] = JoinCase;
			//VehTargetLane[current_veh] = current_lane - 2;
			VehTargetLane[current_veh] = current_lane - 1;
			if (current_veh==VehToWatch)fout_ncacc<<"VEHID="<<current_veh<<",Time="<<timestep<<",currlane="<<current_lane<<",JoinCase:"<<JoinCase<<endl;
		}else{
			active_lane_Change = 0;
			VehStatus[current_veh] = JoinCase;
			VehTargetLane[current_veh]=current_lane-1;
		}
			break;
		case 3:
			active_lane_Change = 0;
			VehStatus[current_veh] = JoinCase;
			VehTargetLane[current_veh] = current_lane;
			//if(Spd_vehicle>0.8*desired_velocity)
				VehTag[current_veh]=1;
			break;
		case 4:
			if ((lateral_pos!=0)){//修正此处可能会报错的情况
				active_lane_Change =+1;
				rel_target_lane =+1; //-1是往右！
				desired_lane_angle = active_lane_Change * lane_angle;
				VehStatus[current_veh] = JoinCase;
				VehTargetLane[current_veh] = current_lane + 1;
				if (current_veh==VehToWatch)fout_ncacc<<"VEHID="<<current_veh<<",Time="<<timestep<<",currlane="<<current_lane<<",JoinCase:"<<JoinCase<<endl;
			}else{
				active_lane_Change = 0;
				VehStatus[current_veh] = JoinCase;
				VehTargetLane[current_veh]=current_lane+1;
			}
			break;
		case 5:
			if ((lateral_pos!=0)){
				active_lane_Change =+1;
				rel_target_lane =+1; 
				desired_lane_angle = active_lane_Change * lane_angle;
				VehStatus[current_veh] = JoinCase;
				VehTargetLane[current_veh] = current_lane + 1;
				if (current_veh==VehToWatch)fout_ncacc<<"VEHID="<<current_veh<<",Time="<<timestep<<",currlane="<<current_lane<<",JoinCase:"<<JoinCase<<endl;
			}else{
				active_lane_Change = 0;
				VehStatus[current_veh] = JoinCase;
				VehTargetLane[current_veh]=current_lane+1;
			}
			break;
		case 6:  //6 表示离开队列的情况：RUN'S STUDIO 04/30/2019
			if (current_lane>1){
				active_lane_Change =-1;
				rel_target_lane =-1; 
				desired_lane_angle = active_lane_Change * lane_angle;
				VehStatus[current_veh] = JoinCase;
				//VehTargetLane[current_veh] =1;
				if (current_veh==VehToWatch)fout_ncacc<<"VEHID="<<current_veh<<",Time="<<timestep<<",currlane="<<current_lane<<",lat:"<<lateral_pos<<endl;
			}else{
				active_lane_Change = 0;
				VehStatus[current_veh] = JoinCase;
				VehTargetLane[current_veh]=current_lane;
			}
			break;
		default:
			break;
		}			
	}

	return 0;
}
/*>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> END OF CODE BLOCK #6>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/


/*>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> START OF CODE BLOCK #7>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
//
//

/*
                                   Primary Input/Output for CODE BLOCK #7
  
  Input Variables : int DistLeadingVeh,
					double Acc_vehicle,
					double Spd_vehicle,
					double LeadingVeh_Acc,
					double LeadingVeh_Spd,
					int LeadingVeh_ID, 
					double Headway,
					double MaxAcc_vehicle,
					double desired_acc_zero,
					double desired_velocity,
					double MInCACCSpdBound.  

					
  Output Variables : double dsrd_acc
*/



double GetDesiredAcc(double d,double a1, double v1,double a2,double v2, long lvid,double H,long lvid2,double a3,double v3,double d2) 
{//a1当前车辆加速度，v1 当前车辆速度(M/S2)
//a2前车加速度，v2 前车速度
//d 到前车的距离
//d2 到前面第二辆车的距离
//lvid 前车ID，不存在的时候就是-1
//lv2id
//H：最小车头时距：0.6m
	//double t_=2.5;
	double dsrd_acc = desired_acc_zero;
	double s1 = 0.5*a1*t_*t_+v1*t_;//t时刻内本车走过的距离
	double s2 = 0.5*a2*t_*t_ + v2*t_;//t时刻内头车走过的距离
	double s3 = 0.5*a3*t_*t_+v3*t_;//t 时刻内后车走过的距离
	double x = d-s1+s2;//t_时刻后两车的相对距离
	double x3= d+s1-s3;
	double dd = 0.0;
	double v = v1;
	double x0 = d;
	double hw = 0.0;//t_时刻后两车的相对时间距离
	double hw_latter=0.0;
	
	//double  lead_vehicle_minimum_time = 0.6; // T 安全时距
	/*long lvid_Left=-1;
	long lvid_Right=-1;
	bool p_Left=false;
	bool p_Right=false;
	double d_Left=0.0;
	double d_Right=0.0;
	double v_Left=0.0;
	double v_Right=0.0;
	double p3=0.0;
	double p4=0.0;
	double sigma_Left=0.0;
	double sigma_Right=0.0;
	 desired_acc_inc = MaxAcc_vehicle;
	 d=(lvid==-1)?100000:d;
	 d2=(lvid2==-1)?100000:d2;
	 double sigma1=d>0?(abs(v1-v2)/sqrt(d)):0;
	 double sigma2=d2>0?(abs(v1-v3)/sqrt(d2)):0;
	 double sigma=sigma1+sigma2;
	 //double p1=sigma>0?(sigma1/(sigma1+sigma2)):1;
	 //double p2=sigma>0?(sigma2/(sigma1+sigma2)):1;
	 if (current_lane<lanes_current_link){
		lvid_Left=AdjVehicles[current_lane+1][3];
		d_Left=sqrt(AdjVehiclesDist[current_lane+1][3]*AdjVehiclesDist[current_lane+1][3]+3.75*3.75);
		double v_Left=AdjVehiclesSpeed[current_lane+1][3];
		sigma_Left=(lvid_Left==-1)?0:0.001*(abs(v1-v_Left)/sqrt(d_Left));
		p_Left=true;
		sigma+=sigma_Left;
	 }
	 if (current_lane>1){
		lvid_Right=AdjVehicles[current_lane-1][3];
		d_Right=sqrt(AdjVehiclesDist[current_lane-1][3]*AdjVehiclesDist[current_lane-1][3]+3.75*3.75);
		v_Right=AdjVehiclesSpeed[current_lane-1][3];
		sigma_Right=(lvid_Right==-1)?0:0.001*(abs(v1-v_Right)/sqrt(d_Right));
		sigma+=sigma_Right;
		p_Right=true;
	 }
	 double p1=sigma1/sigma;
	 double p2=sigma2/sigma;
	 if (p_Left) p3=sigma_Left/sigma;
	 if (p_Right) p4=sigma_Right/sigma;*/

	 
	 desired_acc_inc = MaxAcc_vehicle;
	 d=(lvid==-1)?100000:d;
	 double desired_distance= H*v1+v1*(v1-v2)/2.0/sqrt(MaxAcc_vehicle*desired_dec_inc); //s*=s0+va*T+va*△va/2/sqrt(am*b) 这里am、b都是
	 desired_distance=desired_distance>0?desired_distance:0; //VT+V?V/2sqrtab ?0??
	 //desired_distance+=minimum_gap_distance+AdjVehiclesWidth[2][3];
	 desired_distance+=minimum_gap_distance;
	 
	 desired_acc_inc= MaxAcc_vehicle*(1-(v1*v1*v1*v1/desired_velocity/desired_velocity/desired_velocity/desired_velocity)-(desired_distance*desired_distance/d/d));
	 
	 desired_acc_inc=desired_acc_inc<MaxAcc_vehicle?desired_acc_inc:MaxAcc_vehicle;
	 desired_acc_inc=desired_acc_inc>desired_dec_inc?desired_acc_inc:desired_dec_inc;
	 s1 = 0.5*desired_acc_inc*acc_time*acc_time+v*acc_time; 
	 s2 = 0.5*a2*t_*t_ + v2*t_;
	 x = x0 -s1+s2;
	 hw = x/v;  //车头时距检验
	 hw_latter=x3/v3; //后车车头时距
	 if(current_veh==VehToWatch)fout_ncacc<<"VEHID="<<current_veh<<",hw="<<hw<<",AdjVehiclesDist="<<d<<",v1="<<v1<<",v2="<<v2<<endl;
	 if(current_veh==VehToWatch)fout_ncacc<<"调整前"<<desired_acc_inc<<endl;
	 while((hw<H)&&(desired_acc_inc>=desired_dec_inc)&&(lvid != -1)){
		 desired_acc_inc-=0.1;//以0.1为步长调整本车加速度
		 s1 = 0.5*desired_acc_inc*t_*t_+v*t_; 
		 s2 = 0.5*a2*t_*t_ + v2*t_;
	     x = x0 -s1+s2;
		 hw = x/v;  //车头时距检验
	 }
	 if (desired_acc_inc<=desired_dec_inc) desired_acc_inc+=0.1;
	 if(current_veh==VehToWatch)fout_ncacc<<"第一次调整"<<desired_acc_inc<<endl;
 
	 while((hw<1.0*H)&&(desired_acc_inc>=MaxDeceleration)&&(lvid != -1))
	 {//emgerency brake
		desired_acc_inc-=0.1;
		s1 = 0.5*desired_acc_inc*t_*t_+v*t_; 
		s2 = 0.5*a2*t_*t_ + v2*t_;
	    x = x0 -s1+s2;
		hw = x/v;  //车头时距检验
	}
	 if (desired_acc_inc<=MaxDeceleration) desired_acc_inc+=0.1;
	 if(current_veh==VehToWatch)fout_ncacc<<"第二次调整"<<desired_acc_inc<<endl;
	//if (desired_acc_inc<0.0&&desired_acc_inc>=-0.1) desired_acc_inc=0;

	 if(current_lane==0&&lvid == -1&&v1<=desired_velocity&&active_lane_Change!=0){
		 desired_acc_inc=MaxAcc_vehicle;//在匝道上，前面没车，速度不够就加速
		 if(current_veh==VehToWatch)fout_ncacc<<"case:1"<<endl;
	 }
	 if(current_lane==0&&to_End_Dist<150&&active_lane_Change!=0&&to_End_Dist>0){
		 desired_acc_inc=desired_dec_inc;//减速以寻找合适时机
		 fout_ncacc<<"VEHID="<<current_veh<<"case:3:减速以寻找合适时机"<<endl;
	  }
	  if(current_veh==VehToWatch)fout_ncacc<<"VEHID="<<current_veh<<",-11：,to_end_distance="<<to_End_Dist<<",v="<<Spd_vehicle<<",lvid="<<lvid<<",dv="<<desired_velocity<<endl;
	if((v1<=0.2)&&(v2>1.0)&&(x0>10.0)){
		desired_acc_inc=0.1;
	}
	if(current_veh==VehToWatch)fout_ncacc<<"最终输出加速度："<<desired_acc_inc<<endl;
	 dsrd_acc=desired_acc_inc;

	return dsrd_acc;
}

/*>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> END OF CODE BLOCK #7>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/

/*==========================================================================*/
/*  Ende of DriverModel.cpp                                                 */
/*==========================================================================*/
