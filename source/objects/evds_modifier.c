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
/// @page EVDS_Solver_Modifier Modifier
///
////////////////////////////////////////////////////////////////////////////////
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "evds.h"


#ifndef DOXYGEN_INTERNAL_STRUCTS
#define EVDS_MODIFIER_TYPE_LINEAR		0
#define EVDS_MODIFIER_TYPE_CIRCULAR		1
#define EVDS_MODIFIER_TYPE_PATTERN		2
typedef struct EVDS_MODIFIER_VARIABLES_TAG {
	int i,j,k;

	int type;
	EVDS_REAL vector1_count;
	EVDS_REAL vector2_count;
	EVDS_REAL vector3_count;
	EVDS_REAL vector1[3];
	EVDS_REAL vector2[3];
	EVDS_REAL vector3[3];

	EVDS_REAL circular_step;
	EVDS_REAL circular_radial_step;
	EVDS_REAL circular_normal_step;
	EVDS_REAL circular_arc_length;
	EVDS_REAL circular_radius;
	EVDS_REAL circular_rotate;

	EVDS_REAL u[3]; //Coordinate system of plane in which circular modifier is executed
	EVDS_REAL v[3];
} EVDS_MODIFIER_VARIABLES;
#endif


int EVDS_InternalModifier_Copy(EVDS_MODIFIER_VARIABLES* vars, EVDS_OBJECT* modifier, EVDS_OBJECT* parent, EVDS_OBJECT* object) {
	char name[257] = { 0 }; //Null-terminate the name
	char modifier_suffix[257] = { 0 };
	EVDS_OBJECT* new_object;
	EVDS_SYSTEM* system;
	EVDS_STATE_VECTOR vector;
	EVDS_VECTOR offset;

	//Create name for the object
	EVDS_Object_GetName(object,name,256);
	snprintf(modifier_suffix,256," (%dx%dx%d)",vars->i+1,vars->j+1,vars->k+1);
	strncat(name,modifier_suffix,256);

	//Create a new copy of the object (or return existing copy)
	EVDS_Object_GetSystem(object,&system);
	if (EVDS_System_GetObjectByName(system,name,parent,&new_object) == EVDS_OK) {
		return EVDS_OK;
	} else {
		EVDS_Object_GetStateVector(object,&vector); //Work and transform the original objects position
		EVDS_Object_Copy(object,parent,&new_object);
		EVDS_Object_SetName(new_object,name);
	}

	//Calculate child copies offset
	switch (vars->type) {
		default:
		case EVDS_MODIFIER_TYPE_LINEAR: {
				EVDS_Vector_Set(&offset,EVDS_VECTOR_POSITION,modifier,
					vars->i*vars->vector1[0] + vars->j*vars->vector2[0] + vars->k*vars->vector3[0],
					vars->i*vars->vector1[1] + vars->j*vars->vector2[1] + vars->k*vars->vector3[1],
					vars->i*vars->vector1[2] + vars->j*vars->vector2[2] + vars->k*vars->vector3[2]);
			} break;
		case EVDS_MODIFIER_TYPE_CIRCULAR: {
				EVDS_REAL x = (vars->circular_radius + vars->j*vars->circular_radial_step)*cos(EVDS_RAD(vars->i * vars->circular_step));
				EVDS_REAL y = (vars->circular_radius + vars->j*vars->circular_radial_step)*sin(EVDS_RAD(vars->i * vars->circular_step));

				EVDS_Vector_Set(&offset,EVDS_VECTOR_POSITION,modifier,
					vars->vector2[0] * vars->circular_radius + vars->u[0]*x + vars->v[0]*y + vars->vector1[0]*vars->circular_normal_step*vars->k,
					vars->vector2[1] * vars->circular_radius + vars->u[1]*x + vars->v[1]*y + vars->vector1[1]*vars->circular_normal_step*vars->k,
					vars->vector2[2] * vars->circular_radius + vars->u[2]*x + vars->v[2]*y + vars->vector1[2]*vars->circular_normal_step*vars->k);

				//Rotate the object to align it with the circle
				if (vars->circular_rotate > 0.5) {
					EVDS_VECTOR axis;
					EVDS_QUATERNION delta_quaternion;
					EVDS_Vector_Set(&axis,EVDS_VECTOR_DIRECTION,modifier,
						vars->vector1[0],vars->vector1[1],vars->vector1[2]);
					EVDS_Quaternion_FromVectorAngle(&delta_quaternion,&axis,EVDS_RAD(vars->i*vars->circular_step));

					//Apply rotation
					EVDS_Quaternion_Multiply(&vector.orientation,&delta_quaternion,&vector.orientation);

					//Rotate the local position of the object in modifiers coordinates
					EVDS_Vector_Rotate(&vector.position,&vector.position,&delta_quaternion);
				}
			} break;
	}

	//Apply transformation
	EVDS_Vector_Add(&vector.position,&vector.position,&offset);
	EVDS_Object_SetStateVector(new_object,&vector); //This also converts the state vector from parent coords to new coords

	//Initialize object (because parents children were already initialized
	// prior to modifier initialization)
	EVDS_Object_Initialize(new_object,1);
	return EVDS_OK;
}



////////////////////////////////////////////////////////////////////////////////
/// @brief Initialize modifier and create children copies
////////////////////////////////////////////////////////////////////////////////
int EVDS_InternalModifier_Initialize(EVDS_SYSTEM* system, EVDS_SOLVER* solver, EVDS_OBJECT* object) {
	SIMC_LIST* list;
	SIMC_LIST_ENTRY* entry;
	EVDS_OBJECT* parent;

	//Modifier variables
	EVDS_MODIFIER_VARIABLES vars = { 0 };
	EVDS_VARIABLE* variable;
	char type_str[257] = { 0 };

	//Check type
	if (EVDS_Object_CheckType(object,"modifier") != EVDS_OK) return EVDS_IGNORE_OBJECT; 

	//Read variables
	EVDS_Object_GetRealVariable(object,"vector1.count",&vars.vector1_count,0);
	EVDS_Object_GetRealVariable(object,"vector2.count",&vars.vector2_count,0);
	EVDS_Object_GetRealVariable(object,"vector3.count",&vars.vector3_count,0);
	
	if (vars.vector1_count < 1.0) vars.vector1_count = 1.0;
	if (vars.vector2_count < 1.0) vars.vector2_count = 1.0;
	if (vars.vector3_count < 1.0) vars.vector3_count = 1.0;

	EVDS_Object_GetRealVariable(object,"vector1.x",&vars.vector1[0],0);
	EVDS_Object_GetRealVariable(object,"vector1.y",&vars.vector1[1],0);
	EVDS_Object_GetRealVariable(object,"vector1.z",&vars.vector1[2],0);
	EVDS_Object_GetRealVariable(object,"vector2.x",&vars.vector2[0],0);
	EVDS_Object_GetRealVariable(object,"vector2.y",&vars.vector2[1],0);
	EVDS_Object_GetRealVariable(object,"vector2.z",&vars.vector2[2],0);
	EVDS_Object_GetRealVariable(object,"vector3.x",&vars.vector3[0],0);
	EVDS_Object_GetRealVariable(object,"vector3.y",&vars.vector3[1],0);
	EVDS_Object_GetRealVariable(object,"vector3.z",&vars.vector3[2],0);

	EVDS_Object_GetRealVariable(object,"circular.step",&vars.circular_step,0);
	EVDS_Object_GetRealVariable(object,"circular.radial_step",&vars.circular_radial_step,0);
	EVDS_Object_GetRealVariable(object,"circular.normal_step",&vars.circular_normal_step,0);
	EVDS_Object_GetRealVariable(object,"circular.arc_length",&vars.circular_arc_length,0);
	EVDS_Object_GetRealVariable(object,"circular.radius",&vars.circular_radius,0);
	EVDS_Object_GetRealVariable(object,"circular.rotate",&vars.circular_rotate,0);

	//Fix modifier parameters
	if (vars.circular_step == 0.0) {
		if (vars.circular_arc_length == 0.0) vars.circular_arc_length = 360.0;
		vars.circular_step = vars.circular_arc_length / (vars.vector1_count);
	}

	//Check modifier type, run additional calculations if needed
	vars.type = EVDS_MODIFIER_TYPE_LINEAR;
	if (EVDS_Object_GetVariable(object,"pattern",&variable) == EVDS_OK) {
		EVDS_Variable_GetString(variable,type_str,256,0); //Only read type string if the "pattern" variable exists
	}
	if (strcmp(type_str,"circular") == 0) {
		EVDS_VECTOR normal,direction,vector;
		EVDS_REAL mag;
		vars.type = EVDS_MODIFIER_TYPE_CIRCULAR;

		//Use EVDS vector math
		EVDS_Vector_Set(&normal,   EVDS_VECTOR_DIRECTION,0,vars.vector1[0],vars.vector1[1],vars.vector1[2]);
		EVDS_Vector_Set(&direction,EVDS_VECTOR_DIRECTION,0,vars.vector2[0],vars.vector2[1],vars.vector2[2]);

		//Default values for normal/direction (update vector1/vector2 variables which are used for circular modifier)
		EVDS_Vector_Length(&mag,&normal);
		if (mag == 0.0) {
			EVDS_Vector_Set(&normal,EVDS_VECTOR_DIRECTION,0,1,0,0); //+X normal
			EVDS_Vector_Get(&normal,&vars.vector1[0],&vars.vector1[1],&vars.vector1[2],0);
		}

		EVDS_Vector_Length(&mag,&direction);
		if (mag == 0.0) {
			EVDS_Vector_Set(&direction,EVDS_VECTOR_DIRECTION,0,0,0,1); //+Z direction
			EVDS_Vector_Get(&direction,&vars.vector2[0],&vars.vector2[1],&vars.vector2[2],0);
		}

		//Normalize directions		
		EVDS_Vector_Normalize(&normal,&normal);
		EVDS_Vector_Normalize(&direction,&direction);
		
		//Calculate local coordinate system
		vars.u[0] = -direction.x; //'X' axis
		vars.u[1] = -direction.y;
		vars.u[2] = -direction.z;

		EVDS_Vector_Cross(&vector,&direction,&normal);
		vars.v[0] = vector.x; //'Y' axis completes direction and normal
		vars.v[1] = vector.y;
		vars.v[2] = vector.z;

		/*//Do not generate first ring if radius is zero (only concentric objects)
		if ((circular_radius == 0.0) && (j == 0)) continue;
		//Do not generate first object is radius is non-zero
		if ((circular_radius != 0.0) && (i == 0)) continue;*/
	}

	//Get modifiers parent
	EVDS_Object_GetParent(object,&parent);

	//For every child, create instances
	EVDS_Object_GetAllChildren(object,&list);
	entry = SIMC_List_GetFirst(list);
	while (entry) {
		EVDS_OBJECT* child = (EVDS_OBJECT*)SIMC_List_GetData(list,entry);
		EVDS_Object_Store(child); //Make sure nobody deletes objects data until modifier finishes working with it
		SIMC_List_Stop(list,entry);

		for (vars.i = 0; vars.i < (int)vars.vector1_count; vars.i++) {
			for (vars.j = 0; vars.j < (int)vars.vector2_count; vars.j++) {
				for (vars.k = 0; vars.k < (int)vars.vector3_count; vars.k++) {
					if ((vars.i != 0) || (vars.j != 0) || (vars.k != 0)) {
						EVDS_InternalModifier_Copy(&vars,object,parent,child);
					}
				}
			}
		}

		//Move the child into parent
		EVDS_Object_SetParent(child,parent);
		EVDS_Object_Release(child); //Don't need its data anymore

		//Start again from scratch (list is 1 child less full)
		entry = SIMC_List_GetFirst(list);
	}
	return EVDS_CLAIM_OBJECT;
}




////////////////////////////////////////////////////////////////////////////////
EVDS_SOLVER EVDS_Solver_Modifier = {
	EVDS_InternalModifier_Initialize, //OnInitialize
	0, //OnDeinitialize
	0, //OnSolve
	0, //OnIntegrate
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
int EVDS_Modifier_Register(EVDS_SYSTEM* system) {
	return EVDS_Solver_Register(system,&EVDS_Solver_Modifier);
}
