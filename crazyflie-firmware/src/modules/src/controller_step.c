/*
 *
 * Copyright (c) 2019 Ewoud Smeur and Andre Luis Ogando Paraense
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * This control algorithm is the Incremental Nonlinear Dynamic Inversion (INDI)
 * controller.
 *
 * This is an implementation of the publication in the
 * journal of Control Guidance and Dynamics: Adaptive Incremental Nonlinear
 * Dynamic Inversion for Attitude Control of Micro Aerial Vehicles
 * http://arc.aiaa.org/doi/pdf/10.2514/1.G001490
 */

#include "controller_step.h"

static int16_t cmdThrust_ref = 0;
static int16_t actuatorThrust;
static uint16_t count = 0;

void controllerSTEPInit(void)
{
	count = 0;
}

bool controllerSTEPTest(void)
{
	bool pass = true;

	return pass;
}

void controllerSTEP(control_t *control, setpoint_t *setpoint,
		const sensorData_t *sensors,
		const state_t *state,
		const uint32_t tick)
{

	// stabilezers main loop runs at 1000 hz -> 1 count = 1 ms 

	if(count <= 2500) {
		// wait 2500 counts 
		cmdThrust_ref = 10000;
		control->thrust = cmdThrust_ref;
		control->roll = 0;
		control->pitch = 0;
		control->yaw  = 0;
	}
	else if(count > 2500 && count <= 6500) {
		// wait another 4000 counts
		cmdThrust_ref = 20000;
		control->thrust = cmdThrust_ref;
		control->roll = 0;
		control->pitch = 0;
		control->yaw  = 0;
	}
	else if(count > 6500 && count <= 8000){
		// wait another 1500 counts
		cmdThrust_ref = 5000;
		control->thrust = cmdThrust_ref;
		control->roll = 0;
		control->pitch = 0;
		control->yaw  = 0;
	}
	else {
		control->thrust = 0;
		control->roll = 0;
		control->pitch = 0;
		control->yaw  = 0;
	}

	actuatorThrust = setpoint->thrust;

	count += 1; 

}

LOG_GROUP_START(STEP)
LOG_ADD(LOG_INT16, cmdThrust_ref, &cmdThrust_ref)
LOG_ADD(LOG_INT16, actuatorThrust, &actuatorThrust)
LOG_ADD(LOG_UINT16, count, &count)
LOG_GROUP_STOP(STEP)
