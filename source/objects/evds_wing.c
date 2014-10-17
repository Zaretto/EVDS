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
/// @page EVDS_Solver_Wing Wing
///
////////////////////////////////////////////////////////////////////////////////
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "evds.h"




////////////////////////////////////////////////////////////////////////////////
/// @brief Generate geometry for a single segment of a wing.
////////////////////////////////////////////////////////////////////////////////
int EVDS_InternalWingSegment_GenerateGeometry(EVDS_OBJECT* object) {
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Update state of the wing segment.
////////////////////////////////////////////////////////////////////////////////
int EVDS_InternalWingSegment_Solve(EVDS_SYSTEM* system, EVDS_SOLVER* solver, EVDS_OBJECT* object, EVDS_REAL delta_time) {
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Return forces acting upon a wing segment or control surface.
////////////////////////////////////////////////////////////////////////////////
int EVDS_InternalWingSegment_Integrate(EVDS_SYSTEM* system, EVDS_SOLVER* solver, EVDS_OBJECT* object,
									   EVDS_REAL delta_time, EVDS_STATE_VECTOR* state, EVDS_STATE_VECTOR_DERIVATIVE* derivative) {
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Initialize solver
////////////////////////////////////////////////////////////////////////////////
int EVDS_InternalWingSegment_Initialize(EVDS_SYSTEM* system, EVDS_SOLVER* solver, EVDS_OBJECT* object) {
	if ((EVDS_Object_CheckType(object,"wing.segment") != EVDS_OK) &&
		(EVDS_Object_CheckType(object,"wing.control_surface") != EVDS_OK)) return EVDS_IGNORE_OBJECT;

	//Generate geometry for the wing segment
	//EVDS_InternalWing_GenerateGeometry(object);
	return EVDS_CLAIM_OBJECT;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Update wing state and control surface deflections, flap positions
////////////////////////////////////////////////////////////////////////////////
int EVDS_InternalWing_Solve(EVDS_SYSTEM* system, EVDS_SOLVER* solver, EVDS_OBJECT* object, EVDS_REAL delta_time) {
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Initialize wing geometry and create all segments, control surfaces.
////////////////////////////////////////////////////////////////////////////////
int EVDS_InternalWing_Initialize(EVDS_SYSTEM* system, EVDS_SOLVER* solver, EVDS_OBJECT* object) {
	if (EVDS_Object_CheckType(object,"wing") != EVDS_OK) return EVDS_IGNORE_OBJECT;
	return EVDS_CLAIM_OBJECT;
}




////////////////////////////////////////////////////////////////////////////////
EVDS_SOLVER EVDS_Solver_Wing = {
	EVDS_InternalWing_Initialize, //OnInitialize
	0, //OnDeinitialize
	EVDS_InternalWing_Solve, //OnSolve
	0, //OnIntegrate
	0, //OnStateSave
	0, //OnStateLoad
	0, //OnStartup
	0, //OnShutdown
};

EVDS_SOLVER EVDS_Solver_WingSegment = {
	EVDS_InternalWingSegment_Initialize, //OnInitialize
	0, //OnDeinitialize
	EVDS_InternalWingSegment_Solve, //OnSolve
	EVDS_InternalWingSegment_Integrate, //OnIntegrate
	0, //OnStateSave
	0, //OnStateLoad
	0, //OnStartup
	0, //OnShutdown
};

////////////////////////////////////////////////////////////////////////////////
/// @brief Register wing solver
///
/// @param[in] system Pointer to EVDS_SYSTEM
///
/// @returns Error code
/// @retval EVDS_OK Successfully completed
/// @retval EVDS_ERROR_BAD_PARAMETER "system" is null
/// @retval EVDS_ERROR_BAD_STATE Cannot register solvers in current state
////////////////////////////////////////////////////////////////////////////////
int EVDS_Wing_Register(EVDS_SYSTEM* system) {
	EVDS_ERRCHECK(EVDS_Solver_Register(system,&EVDS_Solver_Wing));
	EVDS_ERRCHECK(EVDS_Solver_Register(system,&EVDS_Solver_WingSegment));
	return EVDS_OK;
}
