/**********************************************************************************************
* PID Library (C version)
* Ported from Arduino PID Library - Version 1.2.1
* by GURobosub
* Initially written by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
*
* This Library is licensed under the MIT License (maybe?)
**********************************************************************************************/

// Define Constants
#define AUTOMATIC	1       // inAuto Variables
#define MANUAL		0
#define DIRECT		0       // controller Direction Variables
#define REVERSE		1
#define P_ON_M		0       // pOnE Variables
#define P_ON_E		1


typedef struct PID {
	float kp;				// * (P)roportional Tuning Parameter
	float ki;				// * (I)ntegral Tuning Parameter
	float kd;				// * (D)erivative Tuning Parameter

	int controllerDirection;
	int pOn;

	float *myInput;		// * Pointers to the Input, Output, and Setpoint variables
	float *myOutput;		//   This creates a hard link between the variables and the
	float *mySetpoint;		//   PID, freeing the user from having to constantly tell us
							//   what these values are.  with pointers we'll just know.

	unsigned long lastTime;
	float outputSum, lastInput;
	unsigned long SampleTime;

	float outMin, outMax;
	int inAuto;
	int pOnE;
} PID;


int PID_Compute(PID * pid) {

	if (!(*pid).inAuto) return 0;	// Leave function if we do not want to run PID

									/* Compute all the working error variables */
	float input = *(*pid).myInput;
	float error = *(*pid).mySetpoint - input;
	float dInput = (input - (*pid).lastInput);
	(*pid).outputSum += ((*pid).ki * error);

	/* Add Proportional on Measurement, if P_ON_M is specified */
	if (!(*pid).pOnE) (*pid).outputSum -= (*pid).kp * dInput;

	// Check if within bounds and saturate if not so
	if ((*pid).outputSum > (*pid).outMax) {
		(*pid).outputSum = (*pid).outMax;
	}
	else if ((*pid).outputSum < (*pid).outMin) {
		(*pid).outputSum = (*pid).outMin;
	}

	/* Add Proportional on Error, if P_ON_M is specified */
	double output;
	if ((*pid).pOnE) output = (*pid).kp * error;
	else output = 0;

	/* Compute Rest of PID Output */
	output += (*pid).outputSum - (*pid).kd * dInput;

	// Check if within bounds and saturate if not so
	if (output > (*pid).outMax) {
		output = (*pid).outMax;
	}
	else if (output < (*pid).outMin) {
		output = (*pid).outMin;
	}

	// Save output value
	*((*pid).myOutput) = output;

	/* Remember variables for next time*/
	(*pid).lastInput = input;

	return 1;
}

void PID_Initialize(PID pid) {
	pid.outputSum = *pid.myOutput;
	pid.lastInput = *pid.myInput;

	// Check if within bounds and saturate if not so
	if (pid.outputSum > pid.outMax) pid.outputSum = pid.outMax;
	else if (pid.outputSum < pid.outMin) pid.outputSum = pid.outMin;
}

void PID_SetControllerDirection(PID * pid, int Direction) {
	if ((*pid).inAuto && Direction != (*pid).controllerDirection) {
		(*pid).kp = (0 - (*pid).kp);
		(*pid).ki = (0 - (*pid).ki);
		(*pid).kd = (0 - (*pid).kd);
	}
	(*pid).controllerDirection = Direction;
}
