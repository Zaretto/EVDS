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
/// @page EVDS_Solver_Wiring Wiring
///
////////////////////////////////////////////////////////////////////////////////
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "evds.h"




////////////////////////////////////////////////////////////////////////////////
/// @brief Generate geometry for the rocket engine
////////////////////////////////////////////////////////////////////////////////
int EVDS_InternalWiring_GenerateConnectorGeometry(EVDS_OBJECT* object) {
	int i,j;
	EVDS_VARIABLE* geometry;
	EVDS_VARIABLE* variable;

	//Default values
	EVDS_REAL wire_radius = 0;
	EVDS_REAL padding = 0;
	EVDS_REAL count_x = 1;
	EVDS_REAL count_y = 1;

	//Reset the cross-sections for the engine
	if (EVDS_Object_GetVariable(object,"geometry.cross_sections",&geometry) == EVDS_OK) {
		EVDS_Variable_Destroy(geometry);
	}
	EVDS_Object_AddVariable(object,"geometry.cross_sections",EVDS_VARIABLE_TYPE_NESTED,&geometry);

	//Get connector parameters
	EVDS_Object_GetRealVariable(object,"wire.radius",&wire_radius,0);
	EVDS_Object_GetRealVariable(object,"pin_padding",&padding,0);
	EVDS_Object_GetRealVariable(object,"pin_count.x",&count_x,0);
	EVDS_Object_GetRealVariable(object,"pin_count.y",&count_y,0);

	//Sanity checks and limits for inputs
	if (count_x < 1) count_x = 1;
	if (count_y < 1) count_y = 1;

	//Generate geometry

	//TEMP HACK: close up face
	EVDS_Variable_AddNested(geometry,"geometry.cross_sections",EVDS_VARIABLE_TYPE_NESTED,&variable);
	EVDS_Variable_AddFloatAttribute(variable,"r",			0.0,0);
	EVDS_Variable_AddFloatAttribute(variable,"offset",		0.0,0);

	//Back face
	EVDS_Variable_AddNested(geometry,"geometry.cross_sections",EVDS_VARIABLE_TYPE_NESTED,&variable);
	EVDS_Variable_AddFloatAttribute(variable,"rx",		count_x*2*(wire_radius+padding),0);
	EVDS_Variable_AddFloatAttribute(variable,"ry",		count_y*2*(wire_radius+padding),0);
	EVDS_Variable_AddFloatAttribute(variable,"offset",	0.0,0);
	EVDS_Variable_AddAttribute(variable,"type",EVDS_VARIABLE_TYPE_STRING,&variable);
	EVDS_Variable_SetString(variable,"rectangle",9);

	//Front face
	EVDS_Variable_AddNested(geometry,"geometry.cross_sections",EVDS_VARIABLE_TYPE_NESTED,&variable);
	EVDS_Variable_AddFloatAttribute(variable,"rx",		count_x*2*(wire_radius+padding),0);
	EVDS_Variable_AddFloatAttribute(variable,"ry",		count_y*2*(wire_radius+padding),0);
	EVDS_Variable_AddFloatAttribute(variable,"offset",	0.05,0);
	EVDS_Variable_AddAttribute(variable,"type",EVDS_VARIABLE_TYPE_STRING,&variable);
	EVDS_Variable_SetString(variable,"rectangle",9);

	//TEMP HACK: close up face
	EVDS_Variable_AddNested(geometry,"geometry.cross_sections",EVDS_VARIABLE_TYPE_NESTED,&variable);
	EVDS_Variable_AddFloatAttribute(variable,"r",			0.0,0);
	EVDS_Variable_AddFloatAttribute(variable,"offset",		0.0,0);


	//Add every pin
	for (i = 0; i < (int)count_x; i++) {
		for (j = 0; j < (int)count_y; j++) {
			EVDS_Variable_AddNested(geometry,"geometry.cross_sections",EVDS_VARIABLE_TYPE_NESTED,&variable);
			EVDS_Variable_AddFloatAttribute(variable,"r",			0.0,0);
			EVDS_Variable_AddFloatAttribute(variable,"offset",		-padding,0);
			EVDS_Variable_AddFloatAttribute(variable,"offset.x",	(i-count_x*0.5+0.5)*2*(wire_radius+padding),0);
			EVDS_Variable_AddFloatAttribute(variable,"offset.y",	(j-count_y*0.5+0.5)*2*(wire_radius+padding),0);
			EVDS_Variable_AddFloatAttribute(variable,"absolute",	1.0,0);

			EVDS_Variable_AddNested(geometry,"geometry.cross_sections",EVDS_VARIABLE_TYPE_NESTED,&variable);
			EVDS_Variable_AddFloatAttribute(variable,"r",			wire_radius,0);
			EVDS_Variable_AddFloatAttribute(variable,"offset",		-padding,0);
			EVDS_Variable_AddFloatAttribute(variable,"offset.x",	(i-count_x*0.5+0.5)*2*(wire_radius+padding),0);
			EVDS_Variable_AddFloatAttribute(variable,"offset.y",	(j-count_y*0.5+0.5)*2*(wire_radius+padding),0);
			EVDS_Variable_AddFloatAttribute(variable,"absolute",	1.0,0);

			EVDS_Variable_AddNested(geometry,"geometry.cross_sections",EVDS_VARIABLE_TYPE_NESTED,&variable);
			EVDS_Variable_AddFloatAttribute(variable,"r",			wire_radius,0);
			EVDS_Variable_AddFloatAttribute(variable,"offset",		0.05+padding,0);
			EVDS_Variable_AddFloatAttribute(variable,"offset.x",	(i-count_x*0.5+0.5)*2*(wire_radius+padding),0);
			EVDS_Variable_AddFloatAttribute(variable,"offset.y",	(j-count_y*0.5+0.5)*2*(wire_radius+padding),0);
			EVDS_Variable_AddFloatAttribute(variable,"absolute",	1.0,0);

			EVDS_Variable_AddNested(geometry,"geometry.cross_sections",EVDS_VARIABLE_TYPE_NESTED,&variable);
			EVDS_Variable_AddFloatAttribute(variable,"r",			0,0);
			EVDS_Variable_AddFloatAttribute(variable,"offset",		0.05+padding,0);
			EVDS_Variable_AddFloatAttribute(variable,"offset.x",	(i-count_x*0.5+0.5)*2*(wire_radius+padding),0);
			EVDS_Variable_AddFloatAttribute(variable,"offset.y",	(j-count_y*0.5+0.5)*2*(wire_radius+padding),0);
			EVDS_Variable_AddFloatAttribute(variable,"absolute",	1.0,0);
		}
	}
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Solver itself
////////////////////////////////////////////////////////////////////////////////
int EVDS_InternalWiring_Solve(EVDS_SYSTEM* system, EVDS_SOLVER* solver, EVDS_OBJECT* object, EVDS_REAL delta_time) {
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Initialize solver
////////////////////////////////////////////////////////////////////////////////
int EVDS_InternalWiring_Initialize(EVDS_SYSTEM* system, EVDS_SOLVER* solver, EVDS_OBJECT* object) {
	if (EVDS_Object_CheckType(object,"wiring.connector") == EVDS_OK) {
		EVDS_InternalWiring_GenerateConnectorGeometry(object);
		return EVDS_CLAIM_OBJECT;
	}
	return EVDS_IGNORE_OBJECT;
}




////////////////////////////////////////////////////////////////////////////////
EVDS_SOLVER EVDS_Solver_Wiring = {
	EVDS_InternalWiring_Initialize, //OnInitialize
	0, //OnDeinitialize
	EVDS_InternalWiring_Solve, //OnSolve
	0, //OnIntegrate
	0, //OnStateSave
	0, //OnStateLoad
	0, //OnStartup
	0, //OnShutdown
};
////////////////////////////////////////////////////////////////////////////////
/// @brief Register wiring solver
///
/// @param[in] system Pointer to EVDS_SYSTEM
///
/// @returns Error code
/// @retval EVDS_OK Successfully completed
/// @retval EVDS_ERROR_BAD_PARAMETER "system" is null
/// @retval EVDS_ERROR_BAD_STATE Cannot register solvers in current state
////////////////////////////////////////////////////////////////////////////////
int EVDS_Wiring_Register(EVDS_SYSTEM* system) {
	return EVDS_Solver_Register(system,&EVDS_Solver_Wiring);
}
