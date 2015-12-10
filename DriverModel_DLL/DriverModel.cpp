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
long    active_lane_change   = 0;
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
ifstream fin;
string str;
char ch;
double val = 0.0;

map<long,int> VehStatus; // 0: false, >1: true
map<long,double> PreviousVehAcc;
map<long,int> VehTargetLane;
map<long,int> VehTag;
map<long,int> VehToAdjustDecForCutIn;
map<long,int> CACCLeader;
map<long,int> HowManyFollwers;
//map<long,int> VehInGroup;


long LeadingVeh_ID =0;
double LeadingVeh_Spd = 0.0;
double LeadingVeh_Acc = 0.0; 
double DistLeadingVeh = 0.0; 
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
//double desired_dec_inc = -5.88;
double init_acc = 0.0001;
double t_ = 0.1;
double acc_time = 30.0; // 3 sec
double dec_time = 30.0; // 5 sec
double uniformspeed_time = 50.0; 
double MaxSpdRegFactor = 1.15; // allow 120% from the desired speed to catch up the leading vehicle. 
//long simple_lanechange = 0;
int JoinCase = 0;
double MinAllowLCDist = 20.0; // in meter
long VehToWatch = 305;
double lane_angle = 0.021;
double VehLengthIndicator = 3.749; // to distinguish CACC vehicle which has 3.749 meters of vehicle length. NOTE: It must be properly configured in VISSIM network from Version 6.

//-- Default setting for adjustable variables. 
double Headway = 0.6; //ADJUSTABLE
double Headway_No = 1.2; //ADJUSTABLE
double MaxLookAheadDist = 300.0; // in meter //ADJUSTABLE
double MInCACCSpdBound = 41.0/3.6; // m/s //ADJUSTABLE
int MinFollower = 5; //ADJUSTABLE
double MaxSpdForCACC = 36.0; //ADJUSTABLE
double desired_dec_inc = -2.88; //ADJUSTABLE 
double desired_dec_inc_lowspd = -5.88; //ADJUSTABLE
double desired_dec_inc_join = -1.5; //ADJUSTABLE
double desired_acc_inc = 0.7; // m/s/s ADJUSTABLE 
long staging_link = 0;

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
      active_lane_change = long_value;
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
      *double_value = desired_velocity;
      return 1;
    case DRIVER_DATA_VEH_COLOR :
      *long_value = vehicle_color;
      return 1;
    case DRIVER_DATA_WANTS_SUGGESTION :
      *long_value = 0;
      return 1;
    case DRIVER_DATA_DESIRED_ACCELERATION :
      *double_value = desired_acceleration;
      return 1;
    case DRIVER_DATA_DESIRED_LANE_ANGLE :
	  *double_value = desired_lane_angle;
      return 1;
    case DRIVER_DATA_ACTIVE_LANE_CHANGE :
      *long_value = active_lane_change;
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
		fin.open("caccconf.dat",std::ios_base::in); 

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
			desired_dec_inc_lowspd = atof(str.c_str());
			fout<<desired_dec_inc_lowspd<<endl;

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

		}
		else
		{
			fout<<"No Configuration Data Exsit. Use Defaults"<<endl;
		}

		fin.close();

      return 1;
    case DRIVER_COMMAND_CREATE_DRIVER :
	    VehStatus[current_veh] = 0;
		PreviousVehAcc[current_veh] = 0;
		VehTargetLane[current_veh] = 0;
		VehTag[current_veh] = 0;
		//VehInGroup[current_veh] = -1;
		VehToAdjustDecForCutIn[current_veh]=0;
		CACCLeader[current_veh] = 0;
		HowManyFollwers[current_veh] = 0;
      return 1;
    case DRIVER_COMMAND_KILL_DRIVER :
		VehStatus.erase(current_veh);
		PreviousVehAcc.erase(current_veh);
		VehTargetLane.erase(current_veh);
		VehTag.erase(current_veh);
		VehToAdjustDecForCutIn.erase(current_veh);
		CACCLeader.erase(current_veh);
		HowManyFollwers.erase(current_veh);
//		VehInGroup.erase(current_veh);
      return 1;
    case DRIVER_COMMAND_MOVE_DRIVER :
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

	  if (lanes_current_link > 3)
	  {
		  //fout<<current_veh<<","<<current_link<<","<<lanes_current_link<<","<<current_lane<<"-->";
		  current_lane = current_lane-(lanes_current_link-3);
		  //fout<<current_lane<<endl;		  
	  }
	  if (lanes_current_link < 3)
	  {
		  current_lane = current_lane+(3-lanes_current_link);
	  }

	  lateral_pos_ind = GetLateralPos(lateral_pos);

	  if (current_veh==VehToWatch)fout_ncacc<<timestep<<",-0,active_lane_Change, current_lane,target_lane,suggestion:"<<active_lane_change<<","<<current_lane<<","<<VehTargetLane[current_veh]<<","<<rel_target_lane<<","<<desired_lane_angle<<",lat:"<<lateral_pos<<endl;

	  //-----------------------------------------------------------------//
  
 
	  if (VehTargetLane[current_veh]==0 || VehTargetLane[current_veh] == current_lane)
	  {
		  if (current_veh==VehToWatch)fout_ncacc<<timestep<<",-2,active_lane_Change, current_lane,target_lane,suggestion:"<<active_lane_change<<","<<current_lane<<","<<VehTargetLane[current_veh]<<","<<rel_target_lane<<","<<desired_lane_angle<<endl;

		  if (lateral_pos_ind !=0)
		  {
				if (lateral_pos_ind>0)
				{
					if (current_veh==VehToWatch)fout_ncacc<<timestep<<",-3,active_lane_Change, current_lane,target_lane,suggestion:"<<active_lane_change<<","<<current_lane<<","<<VehTargetLane[current_veh]<<","<<rel_target_lane<<","<<desired_lane_angle<<","<<endl;
					rel_target_lane =  -1;
					active_lane_change = -1;
					desired_lane_angle = active_lane_change * lane_angle;
				}
				else 
				{
					if (current_veh==VehToWatch)fout_ncacc<<timestep<<",-4,active_lane_Change, current_lane,target_lane,suggestion:"<<active_lane_change<<","<<current_lane<<","<<VehTargetLane[current_veh]<<","<<rel_target_lane<<","<<desired_lane_angle<<","<<endl;
					rel_target_lane =  1;
					active_lane_change = 1;
					desired_lane_angle = active_lane_change * lane_angle;
				}
				if (current_veh==VehToWatch)fout_ncacc<<timestep<<",-5,active_lane_Change, current_lane,target_lane,suggestion:"<<active_lane_change<<","<<current_lane<<","<<VehTargetLane[current_veh]<<","<<rel_target_lane<<","<<desired_lane_angle<<","<<endl;
			return 1;
		  }
		  else
		  {
			if (current_veh==VehToWatch)fout_ncacc<<timestep<<",-6,active_lane_Change, current_lane,target_lane,suggestion:"<<active_lane_change<<","<<current_lane<<","<<VehTargetLane[current_veh]<<","<<rel_target_lane<<","<<desired_lane_angle<<","<<endl;
			active_lane_change = 0;
			desired_lane_angle = 0.0;
			rel_target_lane = 0;
		  }
	  }

	  if (VehStatus[current_veh]>0 && VehTargetLane[current_veh] != current_lane && VehTargetLane[current_veh]>0)
	  {
		  if (current_veh==VehToWatch)fout_ncacc<<timestep<<",-7,active_lane_Change, current_lane,target_lane,suggestion:"<<active_lane_change<<","<<current_lane<<","<<VehTargetLane[current_veh]<<","<<rel_target_lane<<","<<desired_lane_angle<<","<<endl;
		  if (VehStatus[current_veh]<=2)
		  {
			  active_lane_change = -1;
			  rel_target_lane =  -1;
			  desired_lane_angle = active_lane_change * lane_angle;
			  if (current_veh==VehToWatch)fout_ncacc<<timestep<<",-8,active_lane_Change, current_lane,target_lane,suggestion:"<<active_lane_change<<","<<current_lane<<","<<VehTargetLane[current_veh]<<","<<rel_target_lane<<","<<desired_lane_angle<<","<<endl;
		  }
		  if (VehStatus[current_veh]>=4)
		  {
			  active_lane_change = 1;
			  rel_target_lane =  1;
			  desired_lane_angle = active_lane_change * lane_angle;
			  if (current_veh==VehToWatch)fout_ncacc<<timestep<<",-9,active_lane_Change, current_lane,target_lane,suggestion:"<<active_lane_change<<","<<current_lane<<","<<VehTargetLane[current_veh]<<","<<rel_target_lane<<","<<desired_lane_angle<<","<<endl;
		  }

		  return 1;	  
	  }

	 if (current_veh==VehToWatch)fout_ncacc<<timestep<<",-1,active_lane_Change, current_lane,target_lane,suggestion:"<<active_lane_change<<","<<current_lane<<","<<VehTargetLane[current_veh]<<",----"<<vissim_suggestion<<endl;

 

	  if (Spd_vehicle<MInCACCSpdBound) // && VehTag[current_veh]==0 )
	  {
		  //InitArrays();
		  //vissim_suggestion = 1;
		  vehicle_color = RGB(132,134,231)+255*16777216;
		  //active_lane_change = active_lane_change;
		  if (current_veh==VehToWatch)fout_ncacc<<timestep<<",0,lane angle:"<<desired_lane_angle<<","<<active_lane_change<<","<<rel_target_lane<<","<<veh_rel_target_lane<<endl;
		  //return 1;
	  
	  }

	  // The case below happens only when VISSIM provides its lane changing suggestion (vissim_suggestion == 1 && simple_lanechaging ==1). As of July 5, this is obsolute.  
	  if (active_lane_change!=0 && VehTag[current_veh]==0 )
	  {
		  InitArrays();
		  //vissim_suggestion = 1;
		  vehicle_color = RGB(132,134,231)+255*16777216; 
		  //active_lane_change = active_lane_change;
		  //rel_target_lane = active_lane_change;
          //desired_lane_angle = 0.022*active_lane_change;
		  
		  if (current_veh==VehToWatch)fout_ncacc<<timestep<<",0,lane angle:"<<desired_lane_angle<<","<<active_lane_change<<","<<rel_target_lane<<","<<veh_rel_target_lane<<endl;
		  return 1;
	  
	  }

	  // For a CACC vehicle selected to make a room for an incoming CACC vehicle into exisiting platoon. 	
	  if (VehToAdjustDecForCutIn[current_veh]>0)
	  {
		  //if (current_veh==13)
			 // cout<<"";
		  VehToAdjustDecForCutIn[current_veh]=0;
		  if (Spd_vehicle > 0.8*desired_velocity)
		  {
			  desired_acceleration = desired_dec_inc;
	          vehicle_color = RGB(0,255,0)+255*16777216; ;
		  }
		  else
		  {
			  desired_acceleration = desired_acc_zero;
			  vehicle_color = RGB(255,0,0)+255*16777216; ;
		  }

		  InitArrays();	 

		  return 1;
	  }

	  // to make a break point for debug purpose. 	

	  //----------------------------------- main logic ------------------------------------//


	  if (timestep>0.0 && veh_type == 101 )
	  {
		  ControlVehicle();
	  }
	  InitArrays();	  

	  if (current_veh==VehToWatch)fout_ncacc<<timestep<<",-101,active_lane_Change, current_lane,target_lane,suggestion:"<<active_lane_change<<","<<current_lane<<","<<VehTargetLane[current_veh]<<","<<rel_target_lane<<","<<desired_lane_angle<<","<<endl;

      return 1;
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



int ControlVehicle()
{
	/* Obsolute for no suggestion/no simple lane change cae
	if (active_lane_change !=0 && VehTargetLane[current_veh] ==0)
		VehTargetLane[current_veh] = current_lane + active_lane_change; // vissim suggests target lane. 
    */ 

    // if a CACC vehicle runs on the lane that it wants to be or 2) just entered the network but is not willing to change lane and under CACC mode
	//if ((VehTargetLane[current_veh] == current_lane || VehTargetLane[current_veh] ==0) && lateral_pos == 0.0 && VehTag[current_veh]!=0)
    if ((VehTargetLane[current_veh] == current_lane || VehTargetLane[current_veh] ==0) && lateral_pos_ind == 0.0 && VehTag[current_veh]!=0) 
	{
		// initialize or keep the current maneuver. 
		//desired_lane_angle = 0.0; // do not allow to change the group once it was determined. 
		if (current_veh==VehToWatch) fout_ncacc<<timestep<<",2"<<endl;
		 
	}

	if (VehTargetLane[current_veh]>0 && VehTargetLane[current_veh] != current_lane)
	{
		JoinCase = 0;
		if (current_veh==VehToWatch) fout_ncacc<<timestep<<",3,current lane,target lane:"<<current_lane<<","<<VehTargetLane[current_veh]<<endl;
	}

	// To consider a proper leading vehicle

	tlane = 2; // look for the leading vehicle on the current lane 

	// If a CACC vehicle is willing to change lane (VehTargetLane[current_veh]>0) and its target lane is not current lane)

	/*
	if (VehTargetLane[current_veh]>0 && (VehTargetLane[current_veh] == current_lane - 1 || VehTargetLane[current_veh] == current_lane + 1))
	{
		// Check the distances of two leading vehicles on the current lane and the target lane 
		if (current_veh==VehToWatch) fout_ncacc<<timestep<<",4"<<endl;
		tlane = current_lane - 1;	
		if (current_veh==VehToWatch) fout_ncacc<<timestep<<",5"<<endl;
			// if the dist to the current lane is shorter, then set the leading vehice on the current lane
		if (AdjVehiclesDist[2][3] < AdjVehiclesDist[tlane][3])
		{	
			if (current_veh==VehToWatch) fout_ncacc<<timestep<<",6"<<endl;
			tlane = 2;
		}
			// otherwise, the leading vehicle on the target lane is the leading vehicle to consider. 
	}
  	*/

	// ----------  Scan Adjacent Vehicles --------------
	DistLeadingVeh = AdjVehiclesDist[tlane][3]; 
	LeadingVeh_Acc = AdjVehiclesAcc[tlane][3];
	LeadingVeh_Spd = AdjVehiclesSpeed[tlane][3];
	LeadingVeh_ID = AdjVehicles[tlane][3];

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
		if (current_veh==VehToWatch) fout_ncacc<<timestep<<",7"<<endl;
		if (LeadingVeh_ID>0 && DistLeadingVeh<MaxLookAheadDist)
		{
			//Check whether it is an equipped vehicle or not. 

			if (AdjVehiclesWidth[2][3]==VehLengthIndicator)// if equipped,  // Note Nov. 11 2014: if (AdjVehiclesWidth[2][3]!=VehLengthIndicator) for VISSIM 5.4
			{
				if (current_veh==VehToWatch) fout_ncacc<<timestep<<",8"<<endl;
				// Get the optimal accel/decel to keep the headway
				desired_acceleration = GetDesiredAcc(DistLeadingVeh,Acc_vehicle,Spd_vehicle,LeadingVeh_Acc,LeadingVeh_Spd,LeadingVeh_ID, Headway);

				// and tag them as CACC mode
				VehTag[current_veh]=1;
				vehicle_color = RGB(255,0,255)+255*16777216; ; 
				CACCLeader[current_veh] = CACCLeader[LeadingVeh_ID];
				if (WrtFlag)
				{
					fout<<timestep<<","<<veh_od<<",1,"<<current_veh<<","<<LeadingVeh_ID<<endl;

					//VehInGroup[current_veh] = 1;
				}
			}
			else // if not equipped, 
			{   // no CACC vehicle ahead; it works as a ACC with 1.2 seconds of headway 
				if (current_veh==VehToWatch) fout_ncacc<<timestep<<",9"<<endl;
				desired_acceleration = GetDesiredAcc(DistLeadingVeh,Acc_vehicle,Spd_vehicle,LeadingVeh_Acc,LeadingVeh_Spd,LeadingVeh_ID, Headway_No);		

				CACCLeader[current_veh] = current_veh;
				int followers = 0;
				for( map<long,int>::iterator it=CACCLeader.begin(); it!=CACCLeader.end(); ++it)
				{
			       if ((*it).second == current_veh)
					   followers++;
				}

				// tag it as non-CACC mode, which means it is allowed to search for adjacent CACC group

				//fout_ncacc<<timestep<<","<<veh_od<<",0,"<<current_veh<<","<<followers<<endl;

				if (followers <=MinFollower)
					VehTag[current_veh]=0; 
				else
				{
					VehTag[current_veh]=1; 

				}

				if (current_veh==VehToWatch) fout_ncacc<<timestep<<",10,VehTag:"<<VehTag[current_veh]<<endl;


				if (WrtFlag)
					fout<<timestep<<","<<veh_od<<",0,"<<current_veh<<","<<LeadingVeh_ID<<endl;

				vehicle_color = RGB(255,0,255)+255*16777216; ; 

			}
		}
		else // if no vehicle ahead
		{
			// maintain desired speed
			if (Spd_vehicle <= desired_velocity)
				desired_acceleration = MaxAcc_vehicle;
			else
				desired_acceleration = desired_acc_zero;
				//desired_acceleration = GetDesiredAcc(200.0,Acc_vehicle,Spd_vehicle,0.0,Spd_vehicle,-1, Headway_No);		
			if (current_veh==VehToWatch) fout_ncacc<<timestep<<",11,desiredACC"<<desired_acceleration <<endl;

			// tag it as non-CACC mode, which means it is allowed to search for adjacent CACC group
			//VehTag[current_veh]=0;
			CACCLeader[current_veh] = current_veh;

			int followers = 0;
			for( map<long,int>::iterator it=CACCLeader.begin(); it!=CACCLeader.end(); ++it)
			{
			   if ((*it).second == current_veh)
				   followers++;
			}

			// tag it as non-CACC mode, which means it is allowed to search for adjacent CACC group

			//fout_ncacc<<timestep<<","<<veh_od<<",-1,"<<current_veh<<","<<followers<<endl;

			if (followers <=MinFollower)
				VehTag[current_veh]=0; 
			else
			{
				VehTag[current_veh]=1; 
				vehicle_color = RGB(255,0,255)+255*16777216; ; 
			}

			if (current_veh==VehToWatch) fout_ncacc<<timestep<<",12,VehTag:"<<VehTag[current_veh]<<endl;

			if (WrtFlag)
			{
				fout<<timestep<<","<<veh_od<<",-1,"<<current_veh<<","<<LeadingVeh_ID<<endl;
				//VehInGroup[current_veh] = 0;
			}
		}
	}

	// If a CACC vehicle is willing to change lane (VehTargetLane[current_veh]>0) and its target lane is not current lane
	if (VehTargetLane[current_veh] >0 && VehTargetLane[current_veh] != current_lane)
	{
		// if no lane change is in progress but a leading vehicle exisits on the target lane
		if (lateral_pos_ind == 0.0 && LeadingVeh_ID>0)
		{
			if (AdjVehiclesWidth[tlane][3]==VehLengthIndicator) // for VISSIM 5.4 : (AdjVehiclesWidth[tlane][3]!=VehLengthIndicator) 
			{
				// logically, it should not happen as the CACC vehicle in the middle of group is not allowed to change lane
				desired_acceleration = GetDesiredAcc(DistLeadingVeh,Acc_vehicle,Spd_vehicle,LeadingVeh_Acc,LeadingVeh_Spd,LeadingVeh_ID, Headway);		
				if (current_veh==VehToWatch) fout_ncacc<<timestep<<",13"<<endl;
			}
			else
			{
				desired_acceleration = GetDesiredAcc(DistLeadingVeh,Acc_vehicle,Spd_vehicle,LeadingVeh_Acc,LeadingVeh_Spd,LeadingVeh_ID, Headway_No);	
				//VehTag[current_veh] = 0;
				if (current_veh==VehToWatch) fout_ncacc<<timestep<<",14"<<endl;
			}

		}
		// if lane change is in progress but a leading vehicle exisits on the target lane
		if (lateral_pos_ind != 0.0 && tlane !=2 && LeadingVeh_ID>0)
		{
			// if the leading vehile is CACC
			if (AdjVehiclesWidth[tlane][3]==VehLengthIndicator) //// for VISSIM 5.4 : (AdjVehiclesWidth[tlane][3]!=VehLengthIndicator)
			{
				if (current_veh==VehToWatch) fout_ncacc<<timestep<<",15"<<endl;
				desired_acceleration = GetDesiredAcc(DistLeadingVeh,Acc_vehicle,Spd_vehicle,LeadingVeh_Acc,LeadingVeh_Spd,LeadingVeh_ID, Headway);		
			}
			else
			{
				if (current_veh==VehToWatch) fout_ncacc<<timestep<<",16"<<endl;
				desired_acceleration = GetDesiredAcc(DistLeadingVeh,Acc_vehicle,Spd_vehicle,LeadingVeh_Acc,LeadingVeh_Spd,LeadingVeh_ID, Headway_No);		
			}
		}
	}

	// If vehicle is in the CACC group, do not allow lane change. 
	if (VehTag[current_veh]==1 )
	{
		if (current_veh==VehToWatch) fout_ncacc<<timestep<<",17:return. In platoon"<<endl;

		vehicle_color = RGB(255,0,255)+255*16777216; ; 
		//desired_lane_angle = 0.0;
		//vissim_suggestion = 0;
		//simple_lanechange = 0;
		return 1; 
	}

	if (current_link==2 || current_link == 11 || current_link == 4 || current_link == 5)
	{
		if (current_veh==VehToWatch) fout_ncacc<<timestep<<",18:return by link"<<endl;
		return 1;
	}

	//Check: if lane change decision is given by vissim, ignore it. 

	// If a CACC vehicle is not in the CACC group or the most front CACC, and not in the lane change mode, look for adjacent CACC group
	//if(active_lane_change ==0 && lateral_pos==0.0)

/*>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> END OF CODE BLOCK #4>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/


/*>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> START OF CODE BLOCK #5>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
//
//

/*
                                   Primary Input/Output for CODE BLOCK #5
  
  Input Variables : double timestep,
					int active_lane_change,  
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

	if((active_lane_change ==0 || VehTargetLane[current_veh] == current_lane || VehTargetLane[current_veh] ==0) && lateral_pos_ind==0.0)// && VehTag[current_veh]==0)	
	{
		double MinDist = 10000.0;
		double dx1 = 0.0;
		double dx2 = 0.0;
		int decelFlag1 = 0;
		int decelFlag0 = 0;

		if (current_veh==VehToWatch) fout_ncacc<<timestep<<",19"<<endl;
		// Cut-in Join
		 
		for(int i=5;i<=3;i=i+2) // it should start from 1 to get it activated
		{
			dx1 = 0.0;
			dx2 = 0.0;
			// if a CACC group is identified on either right or left lane and both leading and following vehicls on the adjacent lane are all CACCs
			if (JoinCase==0 &&((decelFlag1 + decelFlag0)== 0) && AdjVehicles[i][3]>0 && AdjVehiclesWidth[i][3]==VehLengthIndicator && AdjVehicles[i][1]>0 && AdjVehiclesWidth[i][1]==VehLengthIndicator) // for VISSIM 5.4 : (AdjVehiclesWidth[tlane][3]!=VehLengthIndicator)
			{
				DistTargetLeadingVeh = AdjVehiclesDist[i][3];

				// measure distances from those leading/following vehicles 
				dx1 = AdjVehiclesDist[i][3];
				dx2 = -1.0*AdjVehiclesDist[i][1];

				// if the gap between the following vehicle is not enought, have it decelerate until the gap is > MinAllowLCDist+10
				if (dx2<MinAllowLCDist)
				{
					//VehToAdjustDecForCutIn[AdjVehicles[i][1]]=1;
					decelFlag1 = AdjVehicles[i][1];
				}  

				// if the gaps for both leading and following vehicles are enough, have the subject vehicle change the lane 
				if (dx1>MinAllowLCDist && dx2>MinAllowLCDist )
				{
					JoinCase = i+1;
					MinDist = DistTargetLeadingVeh; 
				}

				if (dx1>0 && dx1<=MinAllowLCDist && Spd_vehicle > 0.8*desired_velocity)
				{
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
					 int active_lane_change,  
					 int rel_target_lane, 
					 double desired_lane_angle,
					 int[] VehStatus,
					 int[] VehTargetLan,
					 double desired_acceleration.
*/
		// Back Join 
		if (((decelFlag1 + decelFlag0)== 0) && JoinCase==0)
		{
			if (current_veh==VehToWatch) fout_ncacc<<timestep<<",20"<<endl;

			for(int i=0;i<=4;i++)
			{
				dx1 = 0.0;
				dx2 = 0.0;
				if (current_veh==VehToWatch) fout_ncacc<<timestep<<",20,"<<AdjVehicles[i][3]<<","<<AdjVehiclesWidth[i][3]<<","<<AdjVehiclesWidth[i][1]<<","<<AdjVehiclesWidth[i][1]<<endl;
				if(AdjVehicles[i][3]>0 && AdjVehiclesWidth[i][3]==VehLengthIndicator && (AdjVehiclesWidth[i][1]!=VehLengthIndicator || AdjVehiclesWidth[i][1]==0)  ) // for VISSIM 5.4 : (AdjVehiclesWidth[tlane][3]!=VehLengthIndicator)
				{
					//JoinCase = i+1;
					DistTargetLeadingVeh = AdjVehiclesDist[i][3];

					if (i<2)
					{
						dx1 = AdjVehiclesDist[1][3];
						dx2 = -1.0*AdjVehiclesDist[1][1];
						if (current_veh==VehToWatch) fout_ncacc<<timestep<<",21"<<endl;
					}
					if (i==2)
					{
						dx1 = MinAllowLCDist+1;
						dx2 = MinAllowLCDist+1;
						if (current_veh==VehToWatch) fout_ncacc<<timestep<<",22"<<endl;
					}

					if (i>2)
					{
						dx1 = AdjVehiclesDist[3][3];
						dx2 = -1.0*AdjVehiclesDist[3][1];
						if (current_veh==VehToWatch) fout_ncacc<<timestep<<",23"<<endl;
					}
					if (dx1>MinAllowLCDist && dx2>MinAllowLCDist && DistTargetLeadingVeh < MinDist )
					{
						JoinCase = i+1;
						MinDist = DistTargetLeadingVeh; // have it select the nearest CACC vehicle. 
						if (current_veh==VehToWatch) fout_ncacc<<timestep<<",24"<<endl;
					}
				 }
			}

			if (JoinCase >0)
			{
				decelFlag0 = 0;
				decelFlag1 = 0;
				if (current_veh==VehToWatch) fout_ncacc<<timestep<<",25: It never happens without cut-in join"<<endl;
				//if (dx1>0 && dx1<=MinAllowLCDist+10 && Spd_vehicle > 0.8*desired_velocity)
				//{
				//	desired_acceleration = desired_dec_inc;
				//}
			}
		}


		if (decelFlag1 >0) 
			VehToAdjustDecForCutIn[decelFlag1]=1;
		if (decelFlag0 >0)
			desired_acceleration = desired_dec_inc;

		if (current_veh==VehToWatch) fout_ncacc<<timestep<<",251, JoinCase:"<<JoinCase<<endl;

		if (current_link == 6)
			JoinCase = 0;

		switch(JoinCase){
		
			if (current_veh==VehToWatch) fout_ncacc<<timestep<<",26, Join:"<<JoinCase<<endl;
		//case 0:
		//	active_lane_change = 0;
		//	break;
		case 1:
			active_lane_change = -1;
			rel_target_lane =  -1;
			desired_lane_angle = active_lane_change * lane_angle;
			VehStatus[current_veh] = JoinCase;
			//VehTargetLane[current_veh] = current_lane - 2;
			VehTargetLane[current_veh] = current_lane - 1;
			break;
		case 2:
			active_lane_change = -1;
			rel_target_lane =  -1;
			desired_lane_angle = active_lane_change * lane_angle;
			VehStatus[current_veh] = JoinCase;
			VehTargetLane[current_veh] = current_lane - 1;
			break;
		case 3:
			active_lane_change = 0;
			VehStatus[current_veh] = JoinCase;
			VehTargetLane[current_veh] = current_lane;
			VehTag[current_veh]=1;
			break;
		case 4:
			active_lane_change = 1;
			rel_target_lane =  1;
			desired_lane_angle = active_lane_change * lane_angle;
			VehStatus[current_veh] = JoinCase;
			VehTargetLane[current_veh] = current_lane + 1;
			break;
		case 5:
			active_lane_change = 1;
			rel_target_lane =  1;
			desired_lane_angle = active_lane_change * lane_angle;
			VehStatus[current_veh] = JoinCase;
			//VehTargetLane[current_veh] = current_lane + 2;
			VehTargetLane[current_veh] = current_lane + 1;
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



double GetDesiredAcc(double d,double a1, double v1,double a2,double v2, long lvid,double H) 
{
	double dsrd_acc = desired_acc_zero;
	double s1 = 0.5*a1*t_*t_+v1*t_;
	double s2 = 0.5*a2*t_*t_ + v2*t_;
	double x = d-s1+s2;
	double dd = 0.0;
	double v = v1;
	double x0 = d;
	double hw = 0.0;

	desired_acc_inc = MaxAcc_vehicle;


	if (lvid == -1 || a1 == 0.0)
	{
		if (v>=desired_velocity)
		{
			if (v>1.05*desired_velocity)
			{
				dsrd_acc = desired_dec_inc;
				if (v1<MInCACCSpdBound)
				  dsrd_acc = desired_dec_inc_lowspd;

			}
			else
				dsrd_acc = desired_acc_zero;
		}
		else
		{
			dsrd_acc = desired_acc_inc;
		}
	}
	else
	{

		bool tag = true;

		//if (a1 == init_acc || a1 == desired_acc_inc )
		if (a1 == init_acc || (a1 != desired_acc_zero && a1 != desired_dec_inc && a1 != desired_dec_inc_lowspd ))
		{
			if (v>=MaxSpdRegFactor*desired_velocity)
			{
				return desired_acc_zero;
			}

			dsrd_acc = desired_acc_inc;
			// Check hw<H in 10 second

			for (int t=0;t<acc_time;t++)
			{
				s1 = 0.5*desired_acc_inc*t_*t_+v*t_; 
				s2 = 0.5*a2*t_*t_ + v2*t_;

				x = x0 -s1+s2;

				hw = x/v; 

				if (hw<H){

					//dsrd_acc = desired_dec_inc;
					tag = false;
					break;
				}
				
				x0=x;
				v=desired_acc_inc*t_+v;
			}

			if (tag == false)
			{
				dsrd_acc = desired_acc_zero;

				for (int t=0;t<uniformspeed_time;t++)
				{
					s1 = 0.5*desired_acc_zero*t_*t_+v*t_; 
					s2 = 0.5*a2*t_*t_ + v2*t_;

					x = x0 -s1+s2;

					hw = x/v; 

					if (hw<H){
						dsrd_acc = desired_dec_inc;
						if (v1<MInCACCSpdBound)
							dsrd_acc = desired_dec_inc_lowspd;
						break;
					}
					
					x0=x;
					v=desired_acc_zero*t_+v;
				}
	
			}
		}

		if (a1 == desired_dec_inc || a1 == desired_dec_inc_lowspd)
		{
			dsrd_acc = desired_acc_zero;
			// Check hw<H in 10 second

			for (int t=0;t<uniformspeed_time;t++)
			{
				s1 = 0.5*desired_acc_zero*t_*t_+v*t_; 
				s2 = 0.5*a2*t_*t_ + v2*t_;

				x = x0 - s1 + s2;
				hw = x/v; 

				if (hw<H){
					dsrd_acc = desired_dec_inc;
					if (v1<MInCACCSpdBound)
						dsrd_acc = desired_dec_inc_lowspd;
					break;
				}
				x0=x;
    			v=desired_acc_zero*t_+v;

			}
		}

		if (a1 == desired_acc_zero)
		{
			dsrd_acc = desired_acc_zero;
			// Check hw<H in 10 second

			for (int t=0;t<uniformspeed_time;t++)
			{
				s1 = 0.5*desired_acc_zero*t_*t_+v*t_; 
				s2 = 0.5*a2*t_*t_ + v2*t_;

				x = x0 - s1 + s2;
				hw = x/v; 

				if (hw<H){
					dsrd_acc = desired_dec_inc;
					if (v1<MInCACCSpdBound)
						dsrd_acc = desired_dec_inc_lowspd;
					break;
				}
				x0=x;
    			v=desired_acc_zero*t_+v;

			}

			if (dsrd_acc == desired_acc_zero)
			{
				if (v>=MaxSpdRegFactor*desired_velocity)
				{
					return desired_acc_zero;
				}
				
				dsrd_acc = desired_acc_inc;

				for (int t=0;t<acc_time;t++)
				{
					s1 = 0.5*desired_acc_inc*t_*t_+v*t_; 
					s2 = 0.5*a2*t_*t_ + v2*t_;

					x = x0 - s1 + s2;
					hw = x/v; 

					if (hw<H){
						dsrd_acc = desired_acc_zero;
						break;
					}
					x0=x;
    				v=desired_acc_inc*t_+v;
				
				}
			}

		}

	}

	return dsrd_acc;
}
/*>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> END OF CODE BLOCK #7>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/

/*==========================================================================*/
/*  Ende of DriverModel.cpp                                                 */
/*==========================================================================*/
