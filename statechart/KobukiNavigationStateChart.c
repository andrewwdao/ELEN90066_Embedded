/*
* KobukiNavigationStatechart.c
*
*/

#include "kobukiNavigationStatechart.h"
#include <math.h>
#include <stdlib.h>
#include "MAFilter.h"
#include "kalmanFilter.h"

// =========================================== additional definitions =========================================== 
// User defined constants
#define	SAFETY_DISTANCE		50	//mm
#define	STANDARD_DISTANCE	200 //mm
#define STANDARD_SPEED		200
#define TURN_SPEED			100
#define STANDARD_ROTATION	80	//deg
#define MAXIMUM_ANGLE		90	//deg - both side
#define LIMIT_TRANSITION_ANGLE	175 //deg
#define ANGLE_TOLERANCE		3	//deg
#define REORIENT_SCALE		5	// MAXIMUM_ANGLE*REORIENT_SCALE has to smaller than 500
#define X_AXIS_LOWER_THRESHOLD	-0.03
#define X_AXIS_UPPER_THRESHOLD	0.03
#define Y_AXIS_UPWARD_LOWER_THRESHOLD	0.07 // straight value: simulation:0.6
#define Y_AXIS_UPWARD_UPPER_THRESHOLD	0.08 
#define Y_AXIS_DOWNWARD_LOWER_THRESHOLD	-0.12 // straight value: simulation:0.11
#define Y_AXIS_DOWNWARD_UPPER_THRESHOLD	-0.10
#define Y_AXIS_FLAT_LOWER_THRESHOLD		-0.04
#define Y_AXIS_FLAT_UPPER_THRESHOLD		-0.02
#define CLIFF_SENSOR_OFFSET		746
#define CLIFF_SENSOR_LOWER_TOLERANCE	2
#define CLIFF_SENSOR_HIGHER_TOLERANCE	20
#define CLIFF_TIMING_THRESHOLD			100 //ms
#define CLIFF_TRANSITION_WAIT_TIME		1	//ms

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
	HILL_REORIENT=0,
	HILL_UPWARD,
	HILL_FORWARD,
	HILL_SAFETY_BACKWARD,
	HILL_LIMIT_TRANSITION,
	HILL_FLAT_CLIFF_TRANSITION,
	HILL_DOWNWARD,
	HILL_FORWARD_SAFTY,
	HILL_HALT
}hillClimbingState_t;

typedef enum{
	NONE=0,
	LOW,
	HIGH
}cliffPossibility_t;
// ============================================================================================================

// Program States
typedef enum{
	INITIAL = 0,						// Initial state
	PAUSE_WAIT_BUTTON_RELEASE,			// Paused; pause button pressed down, wait until released before detecting next press
	UNPAUSE_WAIT_BUTTON_PRESS,			// Paused; wait for pause button to be pressed
	UNPAUSE_WAIT_BUTTON_RELEASE,		// Paused; pause button pressed down, wait until released before returning to previous state
	NAVIGATION,
	HILL_CLIMBING

} robotState_t;


#define DEG_PER_RAD			(180.0 / M_PI)		// degrees per radian
#define RAD_PER_DEG			(M_PI / 180.0)		// radians per degree

// =========================================== additional functions =========================================== 

static void __obstable_handler(int32_t,
							   turn_t*,
							   bool*,
							   navigationState_t*);

static void _navigation_guard_handler(navigationState_t*,
								      int32_t,
							   		  int32_t,
							   		  int32_t*,
							   		  int32_t*,
									  int16_t,
							   		  KobukiSensors_t,
							   		  turn_t*,
							   		  bool*);
static void _navigation_action_handler(navigationState_t,
			  						   int32_t,
									   turn_t,
									   int16_t*,
								       int16_t*);

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
	static robotState_t			unpausedState = NAVIGATION;	// state history for pause region
	static int32_t				distanceAtManeuverStart = 0;	// distance robot had travelled when a maneuver begins, in mm
	static int32_t				angleAtManeuverStart = 0;		// angle through which the robot had turned when a maneuver begins, in deg

	// outputs
	int16_t						leftWheelSpeed = 0;				// speed of the left wheel, in mm/s
	int16_t						rightWheelSpeed = 0;			// speed of the right wheel, in mm/s

	// =========================================== additional variables =========================================== 
	static navigationState_t	navState = FORWARD;
	static hillClimbingState_t	hilState = HILL_REORIENT;
	static turn_t				defaultTurn = RIGHT;
	static int16_t				defaultAngle = 0; // ground direction to head for
	static bool					objHit = false;
	
	static bool					hillNavigationMode = false;
	static bool					startupOnHill = false;
	static bool 				reachEndofHill = false;
	static uint8_t				cliffEdgeCnt = 0;
	static double				buffer_x=0;
	static double				buffer_y=0;
	static cliffPossibility_t	cliffChance = NONE;
	static uint32_t				lastmillisCliff = 0;
	static uint32_t				lastmillisTransition = 0;
	static uint32_t				debounce_cnt = 0;
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
	/******************************************* Navigation ********************************************/
	else if (state == NAVIGATION){
		/* @@@@@@@@@@@@@@@@@@@@@@@@@@ GUARD 1: @@@@@@@@@@@@@@@@@@@@@@@@@@*/
		// buffer_x = KMFilter_x(accelAxes.x);
		buffer_x = MAFilter_x(accelAxes.x);
		if  ((buffer_x < X_AXIS_LOWER_THRESHOLD || 
			buffer_x > X_AXIS_UPPER_THRESHOLD)) {
			state = HILL_CLIMBING;
			hilState = HILL_REORIENT;
			goto state_action;
		}
		
		/* @@@@@@@@@ NAVIGATION GUARD: operate based on navState @@@@@@@@*/
		_navigation_guard_handler(&navState,
								  netDistance,
								  netAngle,
								  &distanceAtManeuverStart,
								  &angleAtManeuverStart,
								  defaultAngle,
								  sensors,
								  &defaultTurn,
								  &objHit);
		/* @@@@@@@@@@@@@@@@@@@@@@@@@ End guard @@@@@@@@@@@@@@@@@@@@@@@@@@@*/


	}/**************************************************************************************************/
	/******************************************* Hill Climbing ********************************************/
	else if (state == HILL_CLIMBING){
		/* 
		..........................................Hill Reorient
		*/
		if (hilState == HILL_REORIENT) {
			/* @@@@@@@@@@@@@@@@@@@@@@@@@@ GUARD 1: @@@@@@@@@@@@@@@@@@@@@@@@@@*/
			// 		Done reorient itself to the direction of the hill
			buffer_y = MAFilter_y(accelAxes.y);
			if  ((buffer_y < Y_AXIS_UPWARD_UPPER_THRESHOLD && 
				buffer_y > Y_AXIS_UPWARD_LOWER_THRESHOLD)) {
				defaultAngle = netAngle;
				hilState = HILL_UPWARD;
			}
			/* @@@@@@@@@@@@@@@@@@@@@@@@@ End guard @@@@@@@@@@@@@@@@@@@@@@@@@@@*/
		}
		    
		/* 
		..........................................Hill Upward
		*/
		else if (hilState == HILL_UPWARD) {
			/* @@@@@@@@@@@@@@@@@@@@@@@@@@ GUARD 1: @@@@@@@@@@@@@@@@@@@@@@@@@@*/
			// 	reached impassible obstacle,
			//  redefine the default angle then change state
			if (sensors.cliffCenter) {
				distanceAtManeuverStart = netDistance;
				hilState = HILL_SAFETY_BACKWARD; //basically just the same as turn
			}
			/* @@@@@@@@@@@@@@@@@@@@@@@@@ End guard @@@@@@@@@@@@@@@@@@@@@@@@@@@*/
		}
		/* 
		..........................................Moving Backward
		*/
		else if (hilState == HILL_SAFETY_BACKWARD) {
				
			/* @@@@@@@@@@@@@@@@@@@@@@@@@@ GUARD 1: @@@@@@@@@@@@@@@@@@@@@@@@@@*/
			// 				after backward a safety distance
			if (netDistance - distanceAtManeuverStart > SAFETY_DISTANCE) {
				defaultAngle = (netAngle>0)?(netAngle-LIMIT_TRANSITION_ANGLE):(netAngle+LIMIT_TRANSITION_ANGLE);
				hilState = HILL_LIMIT_TRANSITION;
				goto state_action;
			/* @@@@@@@@@@@@@@@@@@@@@@@@@ End guard @@@@@@@@@@@@@@@@@@@@@@@@@@@*/
			}
		}
		/* 
		..........................................Hill Limit Transition
		*/
		else if (hilState == HILL_LIMIT_TRANSITION) {
			
			/* @@@@@@@@@@@@@@@@@@@@@@@@@@ GUARD 1: @@@@@@@@@@@@@@@@@@@@@@@@@@*/
			// 		if finish turning to adjust to the new defaultAngle
			if (abs(abs(netAngle) - abs(defaultAngle)) < ANGLE_TOLERANCE &&
				netAngle*defaultAngle >= 0) { //check for the agrement on the sign to make sure it exit at the correct angle
				/* @@@@@@@@@@@@@@@@@@@@@@@@@@ GUARD 1.1: @@@@@@@@@@@@@@@@@@@@@@@@@@*/
				// 				If still on the hill
				buffer_y = MAFilter_y(accelAxes.y);
				if (buffer_y > Y_AXIS_FLAT_UPPER_THRESHOLD || 
					buffer_y < Y_AXIS_FLAT_LOWER_THRESHOLD) {
					hilState = HILL_FLAT_CLIFF_TRANSITION;
				} else {
				/* @@@@@@@@@@@@@@@@@@@@@@@@@@ GUARD 1.2: @@@@@@@@@@@@@@@@@@@@@@@@@@*/
				// 				currently on a flat surface
					hilState = HILL_FORWARD;
				}
				goto state_action;
			}
			/* @@@@@@@@@@@@@@@@@@@@@@@@@ End guard @@@@@@@@@@@@@@@@@@@@@@@@@@@*/
		}
		
		/* 
		..........................................Hill Forward
		*/
		else if (hilState == HILL_FORWARD) {
			
			/* @@@@@@@@@@@@@@@@@@@@@@@@@@ GUARD 1: @@@@@@@@@@@@@@@@@@@@@@@@@@*/
			// 					entering a new cliff
			// buffer_x = KMFilter_x(accelAxes.x);
			buffer_x = MAFilter_x(accelAxes.x);
			if  ((buffer_x < X_AXIS_LOWER_THRESHOLD || 
				buffer_x > X_AXIS_UPPER_THRESHOLD) ||
				(netDistance - distanceAtManeuverStart > 5*SAFETY_DISTANCE)) { // incase there is no flat surface
				hilState = HILL_FLAT_CLIFF_TRANSITION;
				goto state_action;
			}
			/* @@@@@@@@@@@@@@@@@@@@@@@@@ End guard @@@@@@@@@@@@@@@@@@@@@@@@@@@*/
		}
		/* 
		..........................................Hill Flat-Cliff Transition
		*/
		else if (hilState == HILL_FLAT_CLIFF_TRANSITION) {
			/* @@@@@@@@@@@@@@@@@@@@@@@@@@ GUARD 1: @@@@@@@@@@@@@@@@@@@@@@@@@@*/
			// 		Done reorient itself to the direction of the hill
			buffer_y = MAFilter_y(accelAxes.y);
			if  (buffer_y < Y_AXIS_DOWNWARD_UPPER_THRESHOLD && 
				buffer_y > Y_AXIS_DOWNWARD_LOWER_THRESHOLD) {
				defaultAngle = netAngle;
				distanceAtManeuverStart = netDistance;
				hilState = HILL_DOWNWARD;
			}
			/* @@@@@@@@@@@@@@@@@@@@@@@@@ End guard @@@@@@@@@@@@@@@@@@@@@@@@@@@*/
		}
		/* 
		..........................................Hill Downward
		*/
		else if (hilState == HILL_DOWNWARD) {
			/* @@@@@@@@@@@@@@@@@@@@@@@@@@ GUARD 1: @@@@@@@@@@@@@@@@@@@@@@@@@@*/
			// 			detected that the cliff has been exited
			// buffer_x = KMFilter_x(accelAxes.x);
			buffer_y = MAFilter_y(accelAxes.y);
			if  (buffer_y < Y_AXIS_FLAT_UPPER_THRESHOLD && 
				 buffer_y > Y_AXIS_FLAT_LOWER_THRESHOLD &&
				 netDistance - distanceAtManeuverStart > SAFETY_DISTANCE) {
					distanceAtManeuverStart = netDistance;
					hilState = HILL_FORWARD_SAFTY;
					goto state_action;
			}
			/* @@@@@@@@@@@@@@@@@@@@@@@@@ End guard @@@@@@@@@@@@@@@@@@@@@@@@@@@*/
		}
		/* 
		..........................................Hill Forward Safety
		*/
		else if (hilState == HILL_FORWARD_SAFTY) {
			/* @@@@@@@@@@@@@@@@@@@@@@@@@@ GUARD 1: @@@@@@@@@@@@@@@@@@@@@@@@@@*/
			// 		after forward a safety distance
			if	(netDistance - distanceAtManeuverStart > SAFETY_DISTANCE) {
				hilState = HILL_HALT;
				goto state_action;
			}
			/* @@@@@@@@@@@@@@@@@@@@@@@@@ End guard @@@@@@@@@@@@@@@@@@@@@@@@@@@*/
		}
		/* 
		..........................................Hill Limit Transition
		*/
		else if (hilState == HILL_HALT) {
			/* @@@@@@@@@@@@@@@@@@@@@@@@@@ GUARD 1: @@@@@@@@@@@@@@@@@@@@@@@@@@*/
			// 			done task, equivalent to (very) soft reset.
			state = INITIAL;
			unpausedState = NAVIGATION;
			navState = FORWARD;
			hilState = HILL_REORIENT;
			defaultTurn = RIGHT;
			defaultAngle = 0;
			cliffEdgeCnt = 0;
			objHit = false;
			/* @@@@@@@@@@@@@@@@@@@@@@@@@ End guard @@@@@@@@@@@@@@@@@@@@@@@@@@@*/
		}
		
	}/**************************************************************************************************/
	

	
	// else, no transitions are taken

	//*****************
	//* ACTION - state actions *
	//*****************
	state_action:
	switch (state){
	case NAVIGATION:
		_navigation_action_handler(navState,netAngle,defaultTurn,&leftWheelSpeed,&rightWheelSpeed);
		break;
	case HILL_CLIMBING:
		switch (hilState)
		{
			case HILL_FORWARD:
			case HILL_FORWARD_SAFTY:
			case HILL_UPWARD:
			case HILL_DOWNWARD:
				leftWheelSpeed = rightWheelSpeed = STANDARD_SPEED;
				break;
			case HILL_REORIENT:
			case HILL_FLAT_CLIFF_TRANSITION:
				leftWheelSpeed = (accelAxes.x>0)?30:-30; //the sign of netAngle will take care of the direction
				rightWheelSpeed = -leftWheelSpeed;
				break;
			
			case HILL_SAFETY_BACKWARD:
			// full speed behind!
			leftWheelSpeed = rightWheelSpeed = -STANDARD_SPEED;
			break;
			
			case HILL_LIMIT_TRANSITION:
				leftWheelSpeed = (defaultAngle >= 0)?(TURN_SPEED):(-TURN_SPEED);
				rightWheelSpeed = -leftWheelSpeed;
				break;
			case HILL_HALT:
			default:
				// Unknown state
				leftWheelSpeed = rightWheelSpeed = 0;
				break;
		}
		break;
	case INITIAL:
	case PAUSE_WAIT_BUTTON_RELEASE:
	case UNPAUSE_WAIT_BUTTON_PRESS:
	case UNPAUSE_WAIT_BUTTON_RELEASE:
	default:
		// in pause mode or unknown state, robot should be stopped
		leftWheelSpeed = rightWheelSpeed = 0;
		break;
	}


	*pLeftWheelSpeed = leftWheelSpeed;
	*pRightWheelSpeed = rightWheelSpeed;
}


static void __obstable_handler(int32_t 				angleAtManeuverStart,
							   turn_t* 				defaultTurn,
							   bool* 				objHit,
							   navigationState_t*	state)
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

static void _navigation_guard_handler(navigationState_t*		navState,
								      int32_t				netDistance,
							   		  int32_t				netAngle,
							   		  int32_t*				distanceAtManeuverStart,
							   		  int32_t*				angleAtManeuverStart,
									  int16_t				defaultAngle,
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
				(abs(netAngle)) > abs(defaultAngle) + ANGLE_TOLERANCE)
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
				*distanceAtManeuverStart = netDistance;
				__obstable_handler(*angleAtManeuverStart, defaultTurn, objHit, navState);
			}
			/* @@@@@@@@@@@@ GUARD 3: transition to BACKWARD @@@@@@@@@@@@*/
			// --- EVENT: bump right
			//			  ir cliff right
			//			  wheeldrop right
			else if (sensors.bumps_wheelDrops.bumpRight ||
					sensors.cliffRight || 
					sensors.bumps_wheelDrops.wheeldropRight)
			{
				*distanceAtManeuverStart = netDistance;
				*defaultTurn = LEFT; //default reaction after bumping right
				// more evaluation before changing state
				__obstable_handler(*angleAtManeuverStart, defaultTurn, objHit, navState);
			}
			/* @@@@@@@@@@@@ GUARD 4: transition to BACKWARD @@@@@@@@@@@@*/
			// --- EVENT: bump left
			//			  ir cliff left
			//			  wheeldrop left
			else if (sensors.bumps_wheelDrops.bumpLeft || 
					sensors.cliffLeft || 
					sensors.bumps_wheelDrops.wheeldropLeft)
			{
				*distanceAtManeuverStart = netDistance;
				*defaultTurn = RIGHT; //default reaction after bumping right
				// more evaluation before changing state
				__obstable_handler(*angleAtManeuverStart, defaultTurn, objHit, navState);
			}
			/* @@@@@@@@@@@@@@@@@@@@@@@ End guard @@@@@@@@@@@@@@@@@@@@@@*/
			
		}
		/* 
		..........................................Moving Backward
		*/
		/* @@@@@@@@@@@@ GUARD 1: transition to REORIENT @@@@@@@@@@@@*/
		else if (*navState == BACKWARD &&
				(netDistance - *distanceAtManeuverStart > SAFETY_DISTANCE)) {
			*navState = REORIENT;
		}
		/* @@@@@@@@@@@@@@@@@@@@@@@ End guard @@@@@@@@@@@@@@@@@@@@@@*/
		/* 
		..........................................Reorient to "Default Direction"
		*/
		/* @@@@@@@@@@@@ GUARD 1+2: transition to TURN or FORWARD @@@@@@@@@@@@*/
		else if (*navState == REORIENT &&
				(abs(netAngle) < abs(defaultAngle) + ANGLE_TOLERANCE) && 
				(netAngle*defaultAngle >= 0)) { //check for the agrement on the sign to make sure it exit at the correct angle
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
				(abs(abs(netAngle) - abs(*angleAtManeuverStart)) > abs(defaultAngle) + STANDARD_ROTATION) &&
				(netAngle*defaultAngle >= 0)) { //check for the agrement on the sign to make sure it exit at the correct angle
			*distanceAtManeuverStart = netDistance;
			*navState = FORWARD;
		}
		/* @@@@@@@@@@@@@@@@@@@@@@@ End guard @@@@@@@@@@@@@@@@@@@@@@*/
}

static void _navigation_action_handler(navigationState_t navState,
			  						   int32_t			  netAngle,
									   turn_t			  defaultTurn,
									   int16_t*		  leftWheelSpeed,
									   int16_t*		  rightWheelSpeed)
{
	switch (navState){
		case FORWARD:
			// full speed ahead!
			*leftWheelSpeed = *rightWheelSpeed = STANDARD_SPEED;
			break;

		case BACKWARD:
			// full speed behind!
			*leftWheelSpeed = *rightWheelSpeed = -STANDARD_SPEED;
			break;
		
		case REORIENT:
			//adaptive speed for the reorientation
			*leftWheelSpeed = netAngle*REORIENT_SCALE; //the sign of netAngle will take care of the direction
			*rightWheelSpeed = -*leftWheelSpeed;
			break;

		case TURN:
			//adaptive speed for the turn
			// uint16_t turnSpeed = abs(netAngle)*REORIENT_SCALE;
			// leftWheelSpeed = (defaultTurn==RIGHT)?(turnSpeed):(-turnSpeed);
			*leftWheelSpeed = (defaultTurn==RIGHT)?(TURN_SPEED):(-TURN_SPEED);
			*rightWheelSpeed = -*leftWheelSpeed;
			break;

		default:
			// Unknown state
			leftWheelSpeed = rightWheelSpeed = 0;
			break;
	}
}
