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
/// @page EVDS_Solver_Planet Planetary Body
///
////////////////////////////////////////////////////////////////////////////////
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "evds.h"




////////////////////////////////////////////////////////////////////////////////
/// @brief Get planet nearest to the object
////////////////////////////////////////////////////////////////////////////////
int EVDS_Planet_GetNearest(EVDS_OBJECT* object, EVDS_OBJECT** p_planet) {
	EVDS_SYSTEM* system;
	SIMC_LIST_ENTRY* entry;
	SIMC_LIST* planets;

	//State vectors
	EVDS_REAL  nearest_distance = 1.0/EVDS_EPS;
	EVDS_OBJECT* nearest_planet = 0;
	EVDS_STATE_VECTOR state;
	EVDS_Object_GetStateVector(object,&state);

	//Get list of planets
	EVDS_Object_GetSystem(object,&system);
	EVDS_System_GetObjectsByType(system,"planet",&planets);
	entry = SIMC_List_GetFirst(planets);
	while (entry) {
		EVDS_REAL distance;
		EVDS_OBJECT* planet = (EVDS_OBJECT*)SIMC_List_GetData(planets,entry);

		//Get state vector
		EVDS_VECTOR direction;
		EVDS_STATE_VECTOR planet_state;
		EVDS_Object_GetStateVector(planet,&planet_state);

		//Calculate distance
		EVDS_Vector_Subtract(&direction,&planet_state.position,&state.position);
		EVDS_Vector_Dot(&distance,&direction,&direction);
		distance = sqrt(distance);

		//Check if smallest
		if (distance < nearest_distance) {
			nearest_distance = distance;
			nearest_planet = planet;
		}

		//Check next planet
		entry = SIMC_List_GetNext(planets,entry);
	}	

	//Write back planet
	if (nearest_planet) {
		*p_planet = nearest_planet;
		return EVDS_OK;
	} else {
		return EVDS_ERROR_NOT_FOUND;
	}
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Update planet position and state
////////////////////////////////////////////////////////////////////////////////
int EVDS_InternalPlanet_Solve(EVDS_SYSTEM* system, EVDS_SOLVER* solver, EVDS_OBJECT* object, EVDS_REAL delta_time) {
	//FIXME: Manual orbital calculations

	//Solve all children
	SIMC_LIST_ENTRY* entry = SIMC_List_GetFirst(object->children);
	while (entry) {
		EVDS_OBJECT* child = (EVDS_OBJECT*)SIMC_List_GetData(object->children,entry);
		EVDS_Object_Solve(child,delta_time);
		entry = SIMC_List_GetNext(object->children,entry);
	}
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Output accelerations for the planet
////////////////////////////////////////////////////////////////////////////////
int EVDS_InternalPlanet_Integrate(EVDS_SYSTEM* system, EVDS_SOLVER* solver, EVDS_OBJECT* object,
								  EVDS_REAL delta_time, EVDS_STATE_VECTOR* state, EVDS_STATE_VECTOR_DERIVATIVE* derivative) {
	EVDS_REAL is_static;
	EVDS_Object_GetRealVariable(object,"is_static",&is_static,0);

	//Apply physics if planet is not static or updated via ephemeris
	if (is_static < 0.5) {
		//Update planets velocity
		EVDS_Vector_Copy(&derivative->velocity,&state->velocity);
		EVDS_Vector_Copy(&derivative->angular_velocity,&state->angular_velocity);

		//Calculate acceleration due to gravity
		EVDS_Environment_GetGravitationalField(system,&state->position,0,&derivative->acceleration);
	}
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Initialize solver
////////////////////////////////////////////////////////////////////////////////
int EVDS_InternalPlanet_Initialize(EVDS_SYSTEM* system, EVDS_SOLVER* solver, EVDS_OBJECT* object) {
	if (EVDS_Object_CheckType(object,"planet") != EVDS_OK) return EVDS_IGNORE_OBJECT; 

	//Add non-optional variables
	EVDS_Object_AddVariable(object,"is_static",EVDS_VARIABLE_TYPE_FLOAT,0);
	return EVDS_CLAIM_OBJECT;
}




////////////////////////////////////////////////////////////////////////////////
EVDS_SOLVER EVDS_Solver_Planet = {
	EVDS_InternalPlanet_Initialize, //OnInitialize
	0, //OnDeinitialize
	EVDS_InternalPlanet_Solve, //OnSolve
	EVDS_InternalPlanet_Integrate, //OnIntegrate
	0, //OnStateSave
	0, //OnStateLoad
	0, //OnStartup
	0, //OnShutdown
};
////////////////////////////////////////////////////////////////////////////////
/// @brief Register planetary body
///
/// @param[in] system Pointer to EVDS_SYSTEM
///
/// @returns Error code
/// @retval EVDS_OK Successfully completed
/// @retval EVDS_ERROR_BAD_PARAMETER "system" is null
/// @retval EVDS_ERROR_BAD_STATE Cannot register solvers in current state
////////////////////////////////////////////////////////////////////////////////
int EVDS_Planet_Register(EVDS_SYSTEM* system) {
	return EVDS_Solver_Register(system,&EVDS_Solver_Planet);
}
