////////////////////////////////////////////////////////////////////////////////
/// @file
////////////////////////////////////////////////////////////////////////////////
/// Copyright (C) 2012-2013, Black Phoenix
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
/// @page EVDS_Solver_Force Linear Force or Torque
///
/// This object simply applies linear force or torque that can be changed in runtime.
////////////////////////////////////////////////////////////////////////////////
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "evds.h"

#include <stdio.h>


#ifndef DOXYGEN_INTERNAL_STRUCTS
typedef struct EVDS_SOLVER_FORCE_USERDATA_TAG {
	EVDS_VARIABLE *force;
	EVDS_VARIABLE *torque;
} EVDS_SOLVER_FORCE_USERDATA;
#endif


////////////////////////////////////////////////////////////////////////////////
/// @brief Initialize solver
////////////////////////////////////////////////////////////////////////////////
int EVDS_InternalForce_Initialize(EVDS_SYSTEM* system, EVDS_SOLVER* solver, EVDS_OBJECT* object) {
	EVDS_SOLVER_FORCE_USERDATA* userdata;
	if ((EVDS_Object_CheckType(object, "force") != EVDS_OK) &&
		(EVDS_Object_CheckType(object, "torque") != EVDS_OK)) return EVDS_IGNORE_OBJECT;

	// Create userdata
	userdata = malloc(sizeof(EVDS_SOLVER_FORCE_USERDATA));
	memset(userdata, 0, sizeof(EVDS_SOLVER_FORCE_USERDATA));
	EVDS_ERRCHECK(EVDS_Object_SetSolverdata(object, userdata));

	// Get magnitude of force and torque to be applied at this object
	EVDS_Object_AddVariable(object, "force", EVDS_VECTOR_FORCE, &userdata->force);
	EVDS_Object_AddVariable(object, "torque", EVDS_VECTOR_TORQUE, &userdata->torque);
	if (EVDS_Object_CheckType(object, "force") == EVDS_OK) {
		EVDS_Object_AddVariable(object, "magnitude", EVDS_VECTOR_FORCE, &userdata->force);
	} else {
		EVDS_Object_AddVariable(object, "magnitude", EVDS_VECTOR_FORCE, &userdata->torque);
	}
	return EVDS_CLAIM_OBJECT;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Deinitialize engine solver
////////////////////////////////////////////////////////////////////////////////
int EVDS_InternalForce_Deinitialize(EVDS_SYSTEM* system, EVDS_SOLVER* solver, EVDS_OBJECT* object) {
	EVDS_SOLVER_FORCE_USERDATA* userdata;
	EVDS_ERRCHECK(EVDS_Object_GetSolverdata(object, &userdata));
	free(userdata);
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Output engine forces
////////////////////////////////////////////////////////////////////////////////
int EVDS_InternalForce_Integrate(EVDS_SYSTEM* system, EVDS_SOLVER* solver, EVDS_OBJECT* object,
	EVDS_REAL delta_time, EVDS_STATE_VECTOR* state, EVDS_STATE_VECTOR_DERIVATIVE* derivative) {
	EVDS_SOLVER_FORCE_USERDATA* userdata;
	EVDS_ERRCHECK(EVDS_Object_GetSolverdata(object, &userdata));

	// Apply force and torque
	EVDS_Variable_GetVector(userdata->force, &derivative->force);
	EVDS_Variable_GetVector(userdata->torque, &derivative->torque);

	// Put them in origin of this object
	EVDS_Vector_SetPosition(&derivative->force, object, 0.0, 0.0, 0.0);
	EVDS_Vector_SetPosition(&derivative->torque, object, 0.0, 0.0, 0.0);
	return EVDS_OK;
}




////////////////////////////////////////////////////////////////////////////////
EVDS_SOLVER EVDS_Solver_Force = {
	EVDS_InternalForce_Initialize, //OnInitialize
	EVDS_InternalForce_Deinitialize, //OnDeinitialize
	0, //OnSolve
	EVDS_InternalForce_Integrate, //OnIntegrate
	0, //OnStateSave
	0, //OnStateLoad
	0, //OnStartup
	0, //OnShutdown
};
////////////////////////////////////////////////////////////////////////////////
/// @brief Register engine solver
///
/// @param[in] system Pointer to EVDS_SYSTEM
///
/// @returns Error code
/// @retval EVDS_OK Successfully completed
/// @retval EVDS_ERROR_BAD_PARAMETER "system" is null
/// @retval EVDS_ERROR_BAD_STATE Cannot register solvers in current state
////////////////////////////////////////////////////////////////////////////////
int EVDS_Force_Register(EVDS_SYSTEM* system) {
	return EVDS_Solver_Register(system, &EVDS_Solver_Force);
}
