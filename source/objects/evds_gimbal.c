////////////////////////////////////////////////////////////////////////////////
/// @file
////////////////////////////////////////////////////////////////////////////////
/// Copyright (C) 2012-2015, Black Phoenix
///
/// This program is free software; you can redistribute it and/or modify it under
/// the terms of the GNU Lesser General Public License as published by the Free Software
/// Foundation; either version 2 of the License, or (at your option) any later
/// version.
///
/// This program is distributed in the hope that it will be useful, but WITHOUT
/// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
/// FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more
/// details.
///
/// You should have received a copy of the GNU Lesser General Public License along with
/// this program; if not, write to the Free Software Foundation, Inc., 59 Temple
/// Place - Suite 330, Boston, MA  02111-1307, USA.
///
/// Further information about the GNU Lesser General Public License can also be found on
/// the world wide web at http://www.gnu.org.
////////////////////////////////////////////////////////////////////////////////
/// @page EVDS_Solver_Gimbal Gimballing Platform
///
////////////////////////////////////////////////////////////////////////////////
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "evds.h"


#ifndef DOXYGEN_INTERNAL_STRUCTS
typedef struct EVDS_SOLVER_GIMBAL_USERDATA_TAG {
	EVDS_OBJECT* platform;

	EVDS_VARIABLE* pitch_min;
	EVDS_VARIABLE* pitch_max;
	EVDS_VARIABLE* pitch_rate;
	EVDS_VARIABLE* pitch_bits;
	EVDS_VARIABLE* pitch_command;
	EVDS_VARIABLE* pitch_current;

	EVDS_VARIABLE* yaw_min;
	EVDS_VARIABLE* yaw_max;
	EVDS_VARIABLE* yaw_rate;
	EVDS_VARIABLE* yaw_bits;
	EVDS_VARIABLE* yaw_command;
	EVDS_VARIABLE* yaw_current;

	EVDS_VARIABLE* zero_quaternion;
} EVDS_SOLVER_GIMBAL_USERDATA;
#endif



////////////////////////////////////////////////////////////////////////////////
/// @brief Update planet position and state
////////////////////////////////////////////////////////////////////////////////
int EVDS_InternalGimbal_Solve(EVDS_SYSTEM* system, EVDS_SOLVER* solver, EVDS_OBJECT* object, EVDS_REAL delta_time) {
	//Variables related to gimbal
	EVDS_REAL pitch_min,pitch_max,pitch_rate;
	EVDS_REAL pitch_bits,pitch_command,pitch_current;
	EVDS_REAL yaw_min,yaw_max,yaw_rate;
	EVDS_REAL yaw_bits,yaw_command,yaw_current;
	EVDS_REAL pitch_step;
	EVDS_REAL yaw_step;
	EVDS_QUATERNION zero_quaternion,delta_quaternion;
	EVDS_STATE_VECTOR vector;
	
	//Read current value for all variables
	EVDS_SOLVER_GIMBAL_USERDATA* userdata;
	EVDS_Object_GetSolverdata(object,&userdata);

	EVDS_Variable_GetReal(userdata->pitch_min,		&pitch_min);
	EVDS_Variable_GetReal(userdata->pitch_max,		&pitch_max);
	EVDS_Variable_GetReal(userdata->pitch_rate,		&pitch_rate);
	EVDS_Variable_GetReal(userdata->pitch_bits,		&pitch_bits);
	EVDS_Variable_GetReal(userdata->pitch_command,	&pitch_command);
	EVDS_Variable_GetReal(userdata->pitch_current,	&pitch_current);
											  
	EVDS_Variable_GetReal(userdata->yaw_min,		&yaw_min);
	EVDS_Variable_GetReal(userdata->yaw_max,		&yaw_max);
	EVDS_Variable_GetReal(userdata->yaw_rate,		&yaw_rate);
	EVDS_Variable_GetReal(userdata->yaw_bits,		&yaw_bits);
	EVDS_Variable_GetReal(userdata->yaw_command,	&yaw_command);
	EVDS_Variable_GetReal(userdata->yaw_current,	&yaw_current);

	EVDS_Variable_GetQuaternion(userdata->zero_quaternion,&zero_quaternion);

	//Apply bit count to command (signed)
	if (pitch_bits >= 1.0) {
		if (pitch_bits > 31) pitch_bits = 31;
		pitch_step = (pitch_max - pitch_min) / ((1 << (int)pitch_bits) - 1);
		pitch_command = ((int)(pitch_command / pitch_step+0.5)) * pitch_step;
	}
	if (yaw_bits >= 1.0) {
		if (yaw_bits > 31) pitch_bits = 31;
		yaw_step = (yaw_max - yaw_min) / ((1 << (int)yaw_bits) - 1);
		yaw_command = ((int)(yaw_command / yaw_step+0.5)) * yaw_step;
	}

	//Apply rate
	if (pitch_rate > 0.0) {
		EVDS_REAL delta = delta_time * pitch_rate;
		if (fabs(pitch_current - pitch_command) <= delta) {
			pitch_current = pitch_command;
		} else {
			if (pitch_command < pitch_current) {
				pitch_current -= delta;
			} else {
				pitch_current += delta;
			}
		}
	} else {
		pitch_current = pitch_command;
	}
	if (yaw_rate > 0.0) {
		EVDS_REAL delta = delta_time * yaw_rate;
		if (fabs(yaw_current - yaw_command) <= delta) {
			yaw_current = yaw_command;
		} else {
			if (yaw_command < yaw_current) {
				yaw_current -= delta;
			} else {
				yaw_current += delta;
			}
		}
	} else {
		yaw_current = yaw_command;
	}

	//Apply physical limits
	if ((pitch_max != 0.0) && (pitch_min != 0.0)) {
		if (pitch_current > pitch_max) pitch_current = pitch_max;
		if (pitch_current < pitch_min) pitch_current = pitch_min;
	}
	if ((yaw_max != 0.0) && (yaw_min != 0.0)) {
		if (yaw_current > yaw_max) yaw_current = yaw_max;
		if (yaw_current < yaw_min) yaw_current = yaw_min;
	}

	//Remember position
	EVDS_Variable_SetReal(userdata->pitch_current,	pitch_current);
	EVDS_Variable_SetReal(userdata->yaw_current,	yaw_current);

	//Turn the gimbal platform itself
	EVDS_Object_GetStateVector(userdata->platform,&vector);
	EVDS_Quaternion_FromEuler(&delta_quaternion,zero_quaternion.coordinate_system,0,EVDS_RAD(pitch_current),EVDS_RAD(yaw_current));
	EVDS_Quaternion_Multiply(&vector.orientation,&delta_quaternion,&zero_quaternion);
	EVDS_Object_SetStateVector(userdata->platform,&vector);
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Output accelerations for the planet
////////////////////////////////////////////////////////////////////////////////
int EVDS_InternalGimbal_Integrate(EVDS_SYSTEM* system, EVDS_SOLVER* solver, EVDS_OBJECT* object,
								  EVDS_REAL delta_time, EVDS_STATE_VECTOR* state, EVDS_STATE_VECTOR_DERIVATIVE* derivative) {
	EVDS_SOLVER_GIMBAL_USERDATA* userdata;
	EVDS_Object_GetSolverdata(object,&userdata);

	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Initialize solver
////////////////////////////////////////////////////////////////////////////////
int EVDS_InternalGimbal_Initialize(EVDS_SYSTEM* system, EVDS_SOLVER* solver, EVDS_OBJECT* object) {
	EVDS_SOLVER_GIMBAL_USERDATA* userdata;
	EVDS_STATE_VECTOR vector;
	EVDS_VARIABLE* variable;
	EVDS_OBJECT* parent;
	EVDS_OBJECT* platform;
	if (EVDS_Object_CheckType(object,"gimbal") != EVDS_OK) return EVDS_IGNORE_OBJECT; 

	//Create child object (static body that will collect forces from underlying bodies)
	EVDS_Object_GetParent(object,&parent);
	if (EVDS_Object_CreateBy(object,"Platform",parent,&platform) == EVDS_ERROR_NOT_FOUND) {

		//Create new gimbal platform as a static body
		EVDS_Object_SetType(platform,"static_body");
		if (EVDS_Object_GetVariable(object,"mass",&variable) == EVDS_OK) {
			EVDS_REAL mass;
			EVDS_Variable_GetReal(variable,&mass);
			EVDS_Object_AddRealVariable(platform,"mass",mass,0);
		}

		//Move the platform into position of gimbal
		EVDS_Object_GetStateVector(object,&vector);
		EVDS_Object_SetStateVector(platform,&vector);

		//Move children to that platform
		EVDS_Object_MoveChildren(object,platform);
	}

	//Remove mass from gimbal object. It will not take part in mass calculations, but the platform will
	EVDS_Object_AddRealVariable(object,"mass",0,&variable);
	EVDS_Variable_SetReal(variable,0);

	//Initialize platform
	EVDS_Object_Initialize(platform,1);

	//Create userdata
	userdata = (EVDS_SOLVER_GIMBAL_USERDATA*)malloc(sizeof(EVDS_SOLVER_GIMBAL_USERDATA));
	memset(userdata,0,sizeof(EVDS_SOLVER_GIMBAL_USERDATA));
	userdata->platform = platform;
	EVDS_ERRCHECK(EVDS_Object_SetSolverdata(object,userdata));

	//Add variables to this object
	EVDS_Object_AddRealVariable(object,"pitch.min",0,&userdata->pitch_min);
	EVDS_Object_AddRealVariable(object,"pitch.max",0,&userdata->pitch_max);
	EVDS_Object_AddRealVariable(object,"pitch.rate",0,&userdata->pitch_rate);
	EVDS_Object_AddRealVariable(object,"pitch.bits",0,&userdata->pitch_bits);
	EVDS_Object_AddRealVariable(object,"pitch.command",0,&userdata->pitch_command);
	EVDS_Object_AddRealVariable(object,"pitch.current",0,&userdata->pitch_current);

	EVDS_Object_AddRealVariable(object,"yaw.min",0,&userdata->yaw_min);
	EVDS_Object_AddRealVariable(object,"yaw.max",0,&userdata->yaw_max);
	EVDS_Object_AddRealVariable(object,"yaw.rate",0,&userdata->yaw_rate);
	EVDS_Object_AddRealVariable(object,"yaw.bits",0,&userdata->yaw_bits);
	EVDS_Object_AddRealVariable(object,"yaw.command",0,&userdata->yaw_command);
	EVDS_Object_AddRealVariable(object,"yaw.current",0,&userdata->yaw_current);

	EVDS_Object_AddVariable(object,"zero_quaternion",EVDS_VARIABLE_TYPE_QUATERNION,&userdata->zero_quaternion);
	EVDS_Object_GetStateVector(object,&vector);
	EVDS_Variable_SetQuaternion(userdata->zero_quaternion,&vector.orientation);
	return EVDS_CLAIM_OBJECT;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Deinitialize solver
////////////////////////////////////////////////////////////////////////////////
int EVDS_InternalGimbal_Deinitialize(EVDS_SYSTEM* system, EVDS_SOLVER* solver, EVDS_OBJECT* object) {
	void* userdata;
	EVDS_ERRCHECK(EVDS_Object_GetSolverdata(object,&userdata));
	free(userdata);
	return EVDS_OK;
}




////////////////////////////////////////////////////////////////////////////////
EVDS_SOLVER EVDS_Solver_Gimbal = {
	EVDS_InternalGimbal_Initialize, //OnInitialize
	EVDS_InternalGimbal_Deinitialize, //OnDeinitialize
	EVDS_InternalGimbal_Solve, //OnSolve
	EVDS_InternalGimbal_Integrate, //OnIntegrate
	0, //OnStateSave
	0, //OnStateLoad
	0, //OnStartup
	0, //OnShutdown
};
////////////////////////////////////////////////////////////////////////////////
/// @brief Register gimballing platform
///
/// @param[in] system Pointer to EVDS_SYSTEM
///
/// @returns Error code
/// @retval EVDS_OK Successfully completed
/// @retval EVDS_ERROR_BAD_PARAMETER "system" is null
/// @retval EVDS_ERROR_BAD_STATE Cannot register solvers in current state
////////////////////////////////////////////////////////////////////////////////
int EVDS_Gimbal_Register(EVDS_SYSTEM* system) {
	return EVDS_Solver_Register(system,&EVDS_Solver_Gimbal);
}
