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
/// @page EVDS_Propagator_ForwardEuler Forward Euler Propagator
////////////////////////////////////////////////////////////////////////////////
#include <string.h>
#include <math.h>
#include "evds.h"




////////////////////////////////////////////////////////////////////////////////
/// @brief Forward euler integration method
////////////////////////////////////////////////////////////////////////////////
int EVDS_InternalPropagator_ForwardEuler_Solve(EVDS_SYSTEM* system, EVDS_SOLVER* solver, EVDS_OBJECT* coordinate_system, EVDS_REAL h) {
	SIMC_LIST* children;
	SIMC_LIST_ENTRY* entry;

	//Process all children
	EVDS_Object_GetChildren(coordinate_system,&children);
	entry = SIMC_List_GetFirst(children);
	while (entry) {
		//Initial state, final derivative:
		EVDS_STATE_VECTOR state;							//Initial state (t = 0)
		EVDS_STATE_VECTOR_DERIVATIVE state_derivative;		//Derivative at initial state
		EVDS_OBJECT* object = (EVDS_OBJECT*)SIMC_List_GetData(children,entry);

		// Solve everything inside the child
		if (EVDS_Object_Solve(object,h) != EVDS_OK) {
			// In case there is an error move to the next object in list.
			entry = SIMC_List_GetNext(children,entry);
			continue;
		}

		// Get initial state vector
		EVDS_Object_GetStateVector(object,&state);

		// Find derivative
		EVDS_Object_Integrate(object,h,&state,&state_derivative);

		// Calculate new final state
		EVDS_StateVector_MultiplyByTimeAndAdd(&state,&state,&state_derivative,h);

		// Update object state vector
		EVDS_Object_SetStateVector(object,&state);

		//Move to next object in list
		entry = SIMC_List_GetNext(children,entry);
	}

	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Initialize propagator
////////////////////////////////////////////////////////////////////////////////
int EVDS_InternalPropagator_ForwardEuler_Initialize(EVDS_SYSTEM* system, EVDS_SOLVER* solver, EVDS_OBJECT* object) {
	if (EVDS_Object_CheckType(object,"propagator_forwardeuler") != EVDS_OK) return EVDS_IGNORE_OBJECT; 
	return EVDS_CLAIM_OBJECT;
}




////////////////////////////////////////////////////////////////////////////////
EVDS_SOLVER EVDS_Propagator_ForwardEuler = {
	EVDS_InternalPropagator_ForwardEuler_Initialize, //OnInitialize
	0, //OnDeinitialize
	EVDS_InternalPropagator_ForwardEuler_Solve, //OnSolve
	0, //OnIntegrate
	0, //OnStateSave
	0, //OnStateLoad
	0, //OnStartup
	0, //OnShutdown
};
////////////////////////////////////////////////////////////////////////////////
/// @brief Register propagator solver
///
/// @param[in] system Pointer to EVDS_SYSTEM
///
/// @returns Error code
/// @retval EVDS_OK Successfully completed
/// @retval EVDS_ERROR_BAD_PARAMETER "system" is null
/// @retval EVDS_ERROR_BAD_STATE Cannot register solvers in current state
////////////////////////////////////////////////////////////////////////////////
int EVDS_Propagator_ForwardEuler_Register(EVDS_SYSTEM* system) {
	return EVDS_Solver_Register(system,&EVDS_Propagator_ForwardEuler);
}
