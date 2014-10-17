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
/// @page EVDS_Propagator_Heun Heun Iterative Propagator
////////////////////////////////////////////////////////////////////////////////
#include <string.h>
#include <math.h>
#include "evds.h"




////////////////////////////////////////////////////////////////////////////////
/// @brief Heun propagator-corrector solver
////////////////////////////////////////////////////////////////////////////////
int EVDS_InternalPropagator_Heun_Solve(EVDS_SYSTEM* system, EVDS_SOLVER* solver, EVDS_OBJECT* coordinate_system, EVDS_REAL h) {
	SIMC_LIST* children;
	SIMC_LIST_ENTRY* entry;

	//Process all children
	EVDS_Object_GetChildren(coordinate_system,&children);
	entry = SIMC_List_GetFirst(children);
	while (entry) {
		EVDS_REAL error,mag2;
		EVDS_VECTOR temporary;
		EVDS_STATE_VECTOR state_0; //t = 0
		EVDS_STATE_VECTOR_DERIVATIVE state_derivative_0;
		EVDS_STATE_VECTOR state_1; //t = h
		EVDS_STATE_VECTOR_DERIVATIVE state_derivative_1;
		EVDS_STATE_VECTOR state_1n; //(new state) t = h
		EVDS_STATE_VECTOR_DERIVATIVE state_derivative_0n; //(new derivative) t = 0
		EVDS_OBJECT* object = (EVDS_OBJECT*)SIMC_List_GetData(children,entry);

		// Solve everything inside the child
		if (EVDS_Object_Solve(object,h) != EVDS_OK) {
			// In case there is an error (most likely object is not yet initialized)
			//  move to the next object in list.
			entry = SIMC_List_GetNext(children,entry);
			continue;
		}
		EVDS_Vector_Initialize(temporary);

		// Get initial state vector
		EVDS_Object_GetStateVector(object,&state_0);

		// Calculate derivative at starting point (forward integration)
		EVDS_Object_Integrate(object,0.0,&state_0,&state_derivative_0);

		// Make a forward-integration estimate of final state (predictor)
		EVDS_StateVector_MultiplyByTimeAndAdd(&state_1,&state_0,&state_derivative_0,h);

		// Iterative integration
		error = 1e9;
		while (error > 1e-5) {
			// Calculate derivative in the final state
			EVDS_Object_Integrate(object,h,&state_1,&state_derivative_1);

			// Calculate new derivative at the starting point as average between two derivatives (corrector)
			//d0' = (d0 + d1) / 2
			EVDS_StateVector_Derivative_Initialize(&state_derivative_0n,coordinate_system);
			EVDS_StateVector_Derivative_MultiplyAndAdd(&state_derivative_0n,&state_derivative_0n,&state_derivative_0,0.5);
			EVDS_StateVector_Derivative_MultiplyAndAdd(&state_derivative_0n,&state_derivative_0n,&state_derivative_1,0.5);

			// Calculate new final state
			//s1' = s0 + d0' * dt
			EVDS_StateVector_MultiplyByTimeAndAdd(&state_1n,&state_0,&state_derivative_0n,h);

			// Estimate error (FIXME: better criteria)
			error = 0;
			EVDS_Vector_Subtract(&temporary,&state_1.position,&state_1n.position);
			EVDS_Vector_Dot(&mag2,&temporary,&temporary); error += mag2;
			EVDS_Vector_Subtract(&temporary,&state_1.velocity,&state_1n.velocity);
			EVDS_Vector_Dot(&mag2,&temporary,&temporary); error += mag2;
			error = sqrt(error);

			// Set new final state
			EVDS_StateVector_Copy(&state_1,&state_1n);
		}
	
		// Update object state vector
		EVDS_Object_SetStateVector(object,&state_1);

		//Move to next object in list
		entry = SIMC_List_GetNext(children,entry);
	}

	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Initialize propagator
////////////////////////////////////////////////////////////////////////////////
int EVDS_InternalPropagator_Heun_Initialize(EVDS_SYSTEM* system, EVDS_SOLVER* solver, EVDS_OBJECT* object) {
	if (EVDS_Object_CheckType(object,"propagator_heun") != EVDS_OK) return EVDS_IGNORE_OBJECT; 
	return EVDS_CLAIM_OBJECT;
}




////////////////////////////////////////////////////////////////////////////////
EVDS_SOLVER EVDS_Propagator_Heun = {
	EVDS_InternalPropagator_Heun_Initialize, //OnInitialize
	0, //OnDeinitialize
	EVDS_InternalPropagator_Heun_Solve, //OnSolve
	0, //OnIntegrate
	0, //OnStateSave
	0, //OnStateLoad
	0, //OnStartup
	0, //OnShutdown
};
////////////////////////////////////////////////////////////////////////////////
/// @brief Register Heun propagator solver
///
/// @param[in] system Pointer to EVDS_SYSTEM
///
/// @returns Error code
/// @retval EVDS_OK Successfully completed
/// @retval EVDS_ERROR_BAD_PARAMETER "system" is null
/// @retval EVDS_ERROR_BAD_STATE Cannot register solvers in current state
////////////////////////////////////////////////////////////////////////////////
int EVDS_Propagator_Heun_Register(EVDS_SYSTEM* system) {
	return EVDS_Solver_Register(system,&EVDS_Propagator_Heun);
}
