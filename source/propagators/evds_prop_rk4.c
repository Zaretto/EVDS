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
/// @page EVDS_Propagator_RK4 Runge-Kutta 4th order Propagator
////////////////////////////////////////////////////////////////////////////////
#include <string.h>
#include <math.h>
#include "evds.h"




////////////////////////////////////////////////////////////////////////////////
/// @brief RK4 integration method
////////////////////////////////////////////////////////////////////////////////
int EVDS_InternalPropagator_RK4_Solve(EVDS_SYSTEM* system, EVDS_SOLVER* solver, EVDS_OBJECT* coordinate_system, EVDS_REAL h) {
	SIMC_LIST* children;
	SIMC_LIST_ENTRY* entry;

	//Process all children
	EVDS_Object_GetChildren(coordinate_system,&children);
	entry = SIMC_List_GetFirst(children);
	while (entry) {
		//Initial state, final derivative:
		EVDS_STATE_VECTOR state;							//Initial state (t = 0)
		EVDS_STATE_VECTOR_DERIVATIVE state_derivative;		//Derivative at initial state
		//Variables for RK4:
		EVDS_STATE_VECTOR state_temporary;					//Used in calculations
		EVDS_STATE_VECTOR_DERIVATIVE state_derivative_1;	//Four derivatives for RK4
		EVDS_STATE_VECTOR_DERIVATIVE state_derivative_2;
		EVDS_STATE_VECTOR_DERIVATIVE state_derivative_3;
		EVDS_STATE_VECTOR_DERIVATIVE state_derivative_4;
		EVDS_OBJECT* object = (EVDS_OBJECT*)SIMC_List_GetData(children,entry);

		// Solve everything inside the child
		if (EVDS_Object_Solve(object,h) != EVDS_OK) {
			// In case there is an error move to the next object in list.
			entry = SIMC_List_GetNext(children,entry);
			continue;
		}

		// Get initial state vector
		EVDS_Object_GetStateVector(object,&state);
		
		// f1 = f(0,y)
		EVDS_Object_Integrate(object,0.0,&state,&state_derivative_1);

		// f2 = f(t+0.5*h,y+0.5*h*f1)
		EVDS_StateVector_MultiplyByTimeAndAdd(&state_temporary,&state,&state_derivative_1,0.5*h);
		EVDS_Object_Integrate(object,0.5*h,&state_temporary,&state_derivative_2);

		// f3 = f(t+0.5h,y+0.5*h*f2)
		EVDS_StateVector_MultiplyByTimeAndAdd(&state_temporary,&state,&state_derivative_2,0.5*h);
		EVDS_Object_Integrate(object,0.5*h,&state_temporary,&state_derivative_3);

		// f4 = f(t+h,y+h*f3)
		EVDS_StateVector_MultiplyByTimeAndAdd(&state_temporary,&state,&state_derivative_3,h);
		EVDS_Object_Integrate(object,h,&state_temporary,&state_derivative_4);

		// state = state + h*(1/6 f1 + 1/3 f2 + 1/3 f3 + 1/6 f4)
		EVDS_StateVector_Derivative_Initialize(&state_derivative,coordinate_system);
		EVDS_StateVector_Derivative_MultiplyAndAdd(&state_derivative,&state_derivative,&state_derivative_1,1.0/6.0);
		EVDS_StateVector_Derivative_MultiplyAndAdd(&state_derivative,&state_derivative,&state_derivative_2,1.0/3.0);
		EVDS_StateVector_Derivative_MultiplyAndAdd(&state_derivative,&state_derivative,&state_derivative_3,1.0/3.0);
		EVDS_StateVector_Derivative_MultiplyAndAdd(&state_derivative,&state_derivative,&state_derivative_4,1.0/6.0);

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
int EVDS_InternalPropagator_RK4_Initialize(EVDS_SYSTEM* system, EVDS_SOLVER* solver, EVDS_OBJECT* object) {
	if (EVDS_Object_CheckType(object,"propagator_rk4") != EVDS_OK) return EVDS_IGNORE_OBJECT; 
	return EVDS_CLAIM_OBJECT;
}




////////////////////////////////////////////////////////////////////////////////
EVDS_SOLVER EVDS_Propagator_RK4 = {
	EVDS_InternalPropagator_RK4_Initialize, //OnInitialize
	0, //OnDeinitialize
	EVDS_InternalPropagator_RK4_Solve, //OnSolve
	0, //OnIntegrate
	0, //OnStateSave
	0, //OnStateLoad
	0, //OnStartup
	0, //OnShutdown
};
////////////////////////////////////////////////////////////////////////////////
/// @brief Register RK4 propagator solver
///
/// @param[in] system Pointer to EVDS_SYSTEM
///
/// @returns Error code
/// @retval EVDS_OK Successfully completed
/// @retval EVDS_ERROR_BAD_PARAMETER "system" is null
/// @retval EVDS_ERROR_BAD_STATE Cannot register solvers in current state
////////////////////////////////////////////////////////////////////////////////
int EVDS_Propagator_RK4_Register(EVDS_SYSTEM* system) {
	return EVDS_Solver_Register(system,&EVDS_Propagator_RK4);
}
