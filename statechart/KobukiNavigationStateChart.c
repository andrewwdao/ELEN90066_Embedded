/*
* KobukiNavigationStatechart.c
*
*/

#include "kobukiNavigationStatechart.h"
#include <math.h>
#include <stdlib.h>

// =========================================== additional definitions =========================================== 
// User defined constants
#define	SAFETY_DISTANCE		50	//mm
#define	STANDARD_DISTANCE	200 //mm
#define STANDARD_SPEED		200
#define TURN_SPEED			100
#define STANDARD_ROTATION	90	//deg
#define MAXIMUM_ANGLE		90	//deg - both side
#define ANGLE_TOLERANCE		5	//deg
#define REORIENT_SCALE		5	// MAXIMUM_ANGLE*REORIENT_SCALE has to smaller than 500
#define X_AXIS_THRESHOLD	0.024
#define Y_AXIS_LOWER_THRESHOLD	0.01
#define Y_AXIS_UPPER_THRESHOLD	0.03

typedef enum{
	LEFT=0,
	RIGHT
} turn_t;

typedef enum{
	FORWARD,							// move straight forward
	BACKWARD,							// move straight backward
	REORIENT,							// reorient to original angle
	TURN								/// Turn
} navigationState_t;

typedef enum{
	HILL_TRANSITION=0,
	HILL_NAVIGATION,
	HILL_LIMIT_TRANSITION,
	HILL_HALT
}hillClimbingState_t;
// ============================================================================================================

// Program States
typedef enum{
	INITIAL = 0,						// Initial state
	PAUSE_WAIT_BUTTON_RELEASE,			// Paused; pause button pressed down, wait until released before detecting next press
	UNPAUSE_WAIT_BUTTON_PRESS,			// Paused; wait for pause button to be pressed
	UNPAUSE_WAIT_BUTTON_RELEASE,		// Paused; pause button pressed down, wait until released before returning to previous state
	HILL_DETECTING,
	NAVIGATION,
	HILL_CLIMBING

} robotState_t;


#define DEG_PER_RAD			(180.0 / M_PI)		// degrees per radian
#define RAD_PER_DEG			(M_PI / 180.0)		// radians per degree

// =========================================== additional functions =========================================== 

static void obstable_handler(int32_t,
							 turn_t*,
							 bool*,
							 navigationState_t*);

static void navigation_guard_handler(navigationState_t*,
								     int32_t,
							   		 int32_t,
							   		 int32_t*,
							   		 int32_t*,
							   		 KobukiSensors_t,
							   		 turn_t*,
							   		 bool*);
// ============================================================================================================

void KobukiNavigationStatechart(
	const int16_t 				maxWheelSpeed,
	const int32_t 				netDistance,
	const int32_t 				netAngle,
	const KobukiSensors_t		sensors,
	const accelerometer_t		accelAxes,
	int16_t * const 			pRightWheelSpeed,
	int16_t * const 			pLeftWheelSpeed,
	const bool					isSimulator
	){

	// local state
	static robotState_t 		state = INITIAL;				// current program state
	static robotState_t			unpausedState = HILL_DETECTING;	// state history for pause region
	static int32_t				distanceAtManeuverStart = 0;	// distance robot had travelled when a maneuver begins, in mm
	static int32_t				angleAtManeuverStart = 0;		// angle through which the robot had turned when a maneuver begins, in deg

	// outputs
	int16_t						leftWheelSpeed = 0;				// speed of the left wheel, in mm/s
	int16_t						rightWheelSpeed = 0;			// speed of the right wheel, in mm/s

	// =========================================== additional variables =========================================== 
	static navigationState_t	navState = FORWARD;
	static hillClimbingState_t	hilState = HILL_TRANSITION;
	static turn_t				defaultTurn = RIGHT;
	static bool					objHit = false;
	static int16_t				defaultAngle = 0; // ground direction to head for

	static bool					hillNavigationMode = false;
	static bool					startupOnHill = false;
	static bool 				reachEndofHill = false;
	static uint8_t				cliffEdgeCnt = 0;
	// ============================================================================================================

	//*****************************************************
	// state data - process inputs                        *
	//*****************************************************



	if (state == INITIAL
		|| state == PAUSE_WAIT_BUTTON_RELEASE
		|| state == UNPAUSE_WAIT_BUTTON_PRESS
		|| state == UNPAUSE_WAIT_BUTTON_RELEASE
		|| sensors.buttons.B0				// pause button
		){
		switch (state){
		case INITIAL:
			// set state data that may change between simulation and real-world
			if (isSimulator){
			}
			else{
			}
			state = UNPAUSE_WAIT_BUTTON_PRESS; // place into pause state
			break;
		case PAUSE_WAIT_BUTTON_RELEASE:
			// remain in this state until released before detecting next press
			if (!sensors.buttons.B0){
				state = UNPAUSE_WAIT_BUTTON_PRESS;
			}
			break;
		case UNPAUSE_WAIT_BUTTON_RELEASE:
			// user pressed 'pause' button to return to previous state
			if (!sensors.buttons.B0){
				state = unpausedState;
			}
			break;
		case UNPAUSE_WAIT_BUTTON_PRESS:
			// remain in this state until user presses 'pause' button
			if (sensors.buttons.B0){
				state = UNPAUSE_WAIT_BUTTON_RELEASE;
			}
			break;
		default:
			// must be in run region, and pause button has been pressed
			unpausedState = state;
			state = PAUSE_WAIT_BUTTON_RELEASE;
			break;
		}
	}
	//*************************************
	// GUARD - state transition           *
	//*************************************
	/******************************************* Inital Hill Detecting ********************************************/
	else if (state == HILL_DETECTING){
		if (accelAxes.x > X_AXIS_THRESHOLD) {
			startupOnHill = true;
			state = HILL_CLIMBING;
			goto state_action;
		}
		startupOnHill = false;
		state = NAVIGATION;
	}/**************************************************************************************************/

	/******************************************* Navigation ********************************************/
	else if (state == NAVIGATION){
		/* @@@@@@@@@@@@ GLOBAL GUARD 1: transition to CLIMB_HILL @@@@@@@@@@@@*/
		// if (abs(accelAxes.x) > X_AXIS_THRESHOLD) {
		// 	cliffEdgeCnt++;
		// 	state = HILL_CLIMBING;
		// 	goto state_action;
		// }
		/* @@@@@@@@@@@@@@@@@@@@@@@ End guard @@@@@@@@@@@@@@@@@@@@@@*/

		navigation_guard_handler(&navState,
								 netDistance,
								 netAngle,
								 &distanceAtManeuverStart,
								 &angleAtManeuverStart,
								 sensors,
								 &defaultTurn,
								 &objHit);

	}/**************************************************************************************************/
	/******************************************* Hill Climbing ********************************************/
	else if (state == HILL_CLIMBING){
		/* 
		..........................................Hill Transition
		*/
		if (hilState == HILL_TRANSITION) {
			/* @@@@@@@@@@@@ GUARD 1: transition to HILL_NAVIGATION @@@@@@@@@@@@*/
			// 				Done reorient itself to the direction of the hill
			if (Y_AXIS_LOWER_THRESHOLD < accelAxes.y || accelAxes.y < Y_AXIS_UPPER_THRESHOLD) {
				defaultAngle = netAngle;
				hillNavigationMode = true;
				navState = FORWARD;
				state = NAVIGATION;
			}
			/* @@@@@@@@@@@@ GUARD 2+3: transition to HILL_NAVIGATION @@@@@@@@@@@@*/
			// 				Done hill enter or hill exit
			else if ((abs(accelAxes.x) < X_AXIS_THRESHOLD) &&
					 (startupOnHill && cliffEdgeCnt==3)	   ||
					 (!startupOnHill&& cliffEdgeCnt==4)) {
				startupOnHill = false;
				angleAtManeuverStart = netAngle;
				hilState = HILL_HALT;
			}
			/* @@@@@@@@@@@@@@@@@@@@@@@@@@@ End guard @@@@@@@@@@@@@@@@@@@@@@@@@@@@*/
		}
		    
		/* 
		..........................................Hill Navigation
		*/
		/* @@@@@@@@@@@@ GUARD 1: transition to HILL_LIMIT_TRANSITION @@@@@@@@@@@@*/
		// 				reached impassible obstacle, redefine the default angle then change state
		else if (hilState == HILL_NAVIGATION &&
		    sensors.cliffCenter) {
			int32_t buffer = defaultAngle + 180; // turn around
			defaultAngle = (buffer>180)?(defaultAngle-180):buffer;
			hilState = HILL_LIMIT_TRANSITION; //basically just the same as turn
		}
		/* @@@@@@@@@@@@ GUARD 2+3: transition to HILL_LIMIT_TRANSITION @@@@@@@@@@@@*/
		// 				Done hill enter or hill exit
		else if (hilState == HILL_NAVIGATION &&
		    abs(accelAxes.x) < X_AXIS_THRESHOLD) { 
			angleAtManeuverStart = netAngle;
			hilState = HILL_TRANSITION;
		}
		/* 
		..........................................Hill Limit Transition
		*/
		/* @@@@@@@@@@@@ GUARD 1: transition to HILL_NAVIGATION @@@@@@@@@@@@*/
		// 				reached impassible obstacle
		else if (hilState == HILL_LIMIT_TRANSITION &&
		    sensors.cliffCenter) {
			int32_t buffer = defaultAngle + 180; // turn around
			defaultAngle = (buffer>180)?(defaultAngle-180):buffer;
			hilState = HILL_NAVIGATION;
		}
	}/**************************************************************************************************/
	

	
	// else, no transitions are taken

	//*****************
	//* ACTION - state actions *
	//*****************
	state_action:
	switch (state){
	case INITIAL:
	case PAUSE_WAIT_BUTTON_RELEASE:
	case UNPAUSE_WAIT_BUTTON_PRESS:
	case UNPAUSE_WAIT_BUTTON_RELEASE:
		// in pause mode, robot should be stopped
		leftWheelSpeed = rightWheelSpeed = 0;
		break;
	case NAVIGATION:
		switch (navState){
			case FORWARD:
				// full speed ahead!
				leftWheelSpeed = rightWheelSpeed = STANDARD_SPEED;
				break;

			case BACKWARD:
				// full speed behind!
				leftWheelSpeed = rightWheelSpeed = -STANDARD_SPEED;
				break;
			
			case REORIENT:
				//adaptive speed for the reorientation
				leftWheelSpeed = netAngle*REORIENT_SCALE; //the sign of netAngle will take care of the direction
				rightWheelSpeed = -leftWheelSpeed;
				break;
			case TURN:
				//adaptive speed for the turn
				// uint16_t turnSpeed = abs(netAngle)*REORIENT_SCALE;
				// leftWheelSpeed = (defaultTurn==RIGHT)?(turnSpeed):(-turnSpeed);
				leftWheelSpeed = (defaultTurn==RIGHT)?(TURN_SPEED):(-TURN_SPEED);
				rightWheelSpeed = -leftWheelSpeed;
				break;
		}
		break;
	case HILL_CLIMBING:
		switch (hilState)
		{
			case HILL_TRANSITION:
				// basically turn
				rightWheelSpeed = accelAxes.y*200*REORIENT_SCALE; //the sign of netAngle will take care of the direction
				leftWheelSpeed = -leftWheelSpeed;
				break;
			case HILL_NAVIGATION:
				// full speed ahead!
				leftWheelSpeed = rightWheelSpeed = 100;
				break;
		case HILL_LIMIT_TRANSITION:
			int32_t buffer = defaultAngle + 180; // turn around
			defaultAngle = (buffer>180)?(defaultAngle-180):buffer;
			break;
		
		default:
			// Unknown state
			leftWheelSpeed = rightWheelSpeed = 0;
			break;
		}
		break;
	case HILL_DETECTING:
	default:
		// Unknown state
		leftWheelSpeed = rightWheelSpeed = 0;
		break;
	}


	*pLeftWheelSpeed = leftWheelSpeed;
	*pRightWheelSpeed = rightWheelSpeed;
}

static void obstable_handler(int32_t 			angleAtManeuverStart,
							 turn_t 			*defaultTurn,
							 bool 				*objHit,
							 navigationState_t	*state)
{
	*objHit = true;
	
	if ((angleAtManeuverStart > MAXIMUM_ANGLE - ANGLE_TOLERANCE) &&
		(*defaultTurn==RIGHT))
		{*defaultTurn = LEFT;}
	else if ((angleAtManeuverStart < -MAXIMUM_ANGLE + ANGLE_TOLERANCE) &&
		(*defaultTurn==LEFT))
		{*defaultTurn = RIGHT;}
	
	*state = BACKWARD; //second branch if not reorient
	// auto go to state_action after this line
}

static void navigation_guard_handler(navigationState_t*		navState,
								     int32_t				netDistance,
							   		 int32_t				netAngle,
							   		 int32_t*				distanceAtManeuverStart,
							   		 int32_t*				angleAtManeuverStart,
							   		 KobukiSensors_t		sensors,
							   		 turn_t*				defaultTurn,
							   		 bool*					objHit)
{
	/* 
		..........................................Moving Forward
		*/
		if (*navState == FORWARD){
			/* @@@@@@@@@@@@ GUARD 1: transition to REORIENT @@@@@@@@@@@@*/
			// --- DISTANCE INTERVAL:
			//						check for correct angle after every fixed distance
			if ((netDistance - *distanceAtManeuverStart > STANDARD_DISTANCE) &&
				(abs(netAngle)) > ANGLE_TOLERANCE)
			{
				*navState = REORIENT;
				return;//goto state_action;
			}
			
			/* @@@@@@@@@@@@ GUARD 2: transition to BACKWARD @@@@@@@@@@@@*/
			// --- EVENT: bump center
			//			  ir cliff center
			else if (sensors.bumps_wheelDrops.bumpCenter ||
					sensors.cliffCenter)
			{
				obstable_handler(*angleAtManeuverStart, defaultTurn, objHit, navState);
			}
			/* @@@@@@@@@@@@ GUARD 3: transition to BACKWARD @@@@@@@@@@@@*/
			// --- EVENT: bump right
			//			  ir cliff right
			//			  wheeldrop right
			else if (sensors.bumps_wheelDrops.bumpRight ||
					sensors.cliffRight || 
					sensors.bumps_wheelDrops.wheeldropRight)
			{
				*defaultTurn = LEFT; //default reaction after bumping right
				// more evaluation before changing state
				obstable_handler(*angleAtManeuverStart, defaultTurn, objHit, navState);
			}
			/* @@@@@@@@@@@@ GUARD 4: transition to BACKWARD @@@@@@@@@@@@*/
			// --- EVENT: bump left
			//			  ir cliff left
			//			  wheeldrop left
			else if (sensors.bumps_wheelDrops.bumpLeft || 
					sensors.cliffLeft || 
					sensors.bumps_wheelDrops.wheeldropLeft)
			{
				*defaultTurn = RIGHT; //default reaction after bumping right
				// more evaluation before changing state
				obstable_handler(*angleAtManeuverStart, defaultTurn, objHit, navState);
			}
			/* @@@@@@@@@@@@@@@@@@@@@@@ End guard @@@@@@@@@@@@@@@@@@@@@@*/
			
		}
		/* 
		..........................................Moving Backward
		*/
		/* @@@@@@@@@@@@ GUARD 1: transition to REORIENT @@@@@@@@@@@@*/
		else if (*navState == BACKWARD &&
				(netDistance - *distanceAtManeuverStart > SAFETY_DISTANCE)) {
			*angleAtManeuverStart = netAngle;
			*distanceAtManeuverStart = netDistance;
			*navState = REORIENT;
		}
		/* @@@@@@@@@@@@@@@@@@@@@@@ End guard @@@@@@@@@@@@@@@@@@@@@@*/
		/* 
		..........................................Reorient to "Default Direction"
		*/
		/* @@@@@@@@@@@@ GUARD 1+2: transition to TURN or FORWARD @@@@@@@@@@@@*/
		else if (*navState == REORIENT &&
				(abs(netAngle)) < ANGLE_TOLERANCE) {
			*angleAtManeuverStart = netAngle;
			*distanceAtManeuverStart = netDistance;
			*navState = *objHit?TURN:FORWARD;
			*objHit = false; //reset the flag
		}
		/* @@@@@@@@@@@@@@@@@@@@@@@ End guard @@@@@@@@@@@@@@@@@@@@@@*/
		/* 
		..........................................Turn
		*/
		/* @@@@@@@@@@@@ GUARD 1: transition to FORWARD @@@@@@@@@@@@*/
		else if (*navState == TURN &&
				(abs(abs(netAngle) - abs(*angleAtManeuverStart)) > STANDARD_ROTATION)) {
			*angleAtManeuverStart = netAngle;
			*distanceAtManeuverStart = netDistance;
			*navState = FORWARD;
		}
		/* @@@@@@@@@@@@@@@@@@@@@@@ End guard @@@@@@@@@@@@@@@@@@@@@@*/
}
