#include "pid.h"
#include "maths.h"

// PID

void initPid(PID* pid)
{
	pid->errorAccum = 0;
	pid->lastError = 0;
	pid->first = true;
}

float evalPid(PID* pid, float error, float dt)
{
	if (pid->first)
	{
		pid->lastError = error;
		pid->first = false;
	}

	if(isnanf(error) == 0)
	{
		pid->errorAccum += error * dt;
	}

	float output = 
		(pid->kP * error) +
		(pid->kI * pid->errorAccum) +
		(pid->kD * (error - pid->lastError) / dt);

	pid->lastError = error;

	return output;
}


// 