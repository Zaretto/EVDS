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
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

#include "evds.h"
#include "sim_xml.h"




////////////////////////////////////////////////////////////////////////////////
/// @brief 
////////////////////////////////////////////////////////////////////////////////
int EVDS_InternalVariable_CompareEntries_Linear(const EVDS_VARIABLE_FVALUE_LINEAR* v1, const EVDS_VARIABLE_FVALUE_LINEAR* v2) {
	if (v1->x > v2->x) return 1;
	if (v1->x < v2->x) return -1;
	return 0;
}

int EVDS_InternalVariable_CompareEntries_Spline(const EVDS_VARIABLE_FVALUE_SPLINE* v1, const EVDS_VARIABLE_FVALUE_SPLINE* v2) {
	if (v1->x > v2->x) return 1;
	if (v1->x < v2->x) return -1;
	return 0;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Initialize function data structure
////////////////////////////////////////////////////////////////////////////////
int EVDS_InternalVariable_InitializeFunction(EVDS_VARIABLE* variable, EVDS_VARIABLE_FUNCTION* function, const char* data) {
	int i;
	SIMC_LIST_ENTRY* entry;
	char *ptr,*end_ptr;
	EVDS_REAL x,value;
	EVDS_REAL avg_value;
	int avg_count;

	//Select interpolation type
	function->interpolation = EVDS_VARIABLE_FUNCTION_INTERPOLATION_LINEAR;

	//Count total number of entries
	function->data_count = 0;
	i = 0;

	//Calculate number of data entries
	if (data) {
		ptr = (char*)data;
		while (1) {
			//Parse strings
			EVDS_StringToReal(ptr,&end_ptr,&x);
			if (end_ptr == ptr) break;
			ptr = end_ptr;

			EVDS_StringToReal(ptr,&end_ptr,&value);
			if (end_ptr == ptr) break;
			ptr = end_ptr;

			//Count an extra entry if both are valid numbers
			function->data_count++;
		}
	}

	//Calculate number of nested functions
	entry = SIMC_List_GetFirst(variable->list);
	while (entry) {
		//FIXME: check if type is "data"
		function->data_count++;
		entry = SIMC_List_GetNext(variable->list,entry);
	}


	//Allocate table
	function->data = malloc(sizeof(EVDS_VARIABLE_FVALUE_LINEAR)*function->data_count);
	memset(function->data,0,sizeof(EVDS_VARIABLE_FVALUE_LINEAR)*function->data_count);

	//Compute average value (to determine constant)
	avg_count = 0;
	avg_value = 0.0;


	//Fill table with data entries
	if (data) {
		ptr = (char*)data;
		while (1) {
			//Parse strings
			EVDS_StringToReal(ptr,&end_ptr,&x);
			if (end_ptr == ptr) break;
			ptr = end_ptr;

			EVDS_StringToReal(ptr,&end_ptr,&value);
			if (end_ptr == ptr) break;
			ptr = end_ptr;

			//Write entry
			function->linear[i].x = x;
			function->linear[i].value = value;
			function->linear[i].function = 0;

			//Accumulate
			avg_value += value;
			avg_count++;
			i++;
		}
	}

	//Compute constant value
	if (avg_count > 0) {
		function->constant_value = avg_value / ((EVDS_REAL)avg_count);
	}

	//Fill table with nested function entries
	entry = SIMC_List_GetFirst(variable->list);
	while (entry) {
		EVDS_VARIABLE* nested_function = SIMC_List_GetData(variable->list,entry);
		EVDS_VARIABLE* x_var;

		//Get X value
		x = 0.0;
		if (EVDS_Variable_GetAttribute(nested_function,"value",&x_var) == EVDS_OK) {
			EVDS_Variable_GetReal(x_var,&x);
		}

		//Get default Y value
		EVDS_Variable_GetReal(nested_function,&value);

		//FIXME: check if type is "data"
		function->linear[i].x = x;
		function->linear[i].value = value;
		function->linear[i].function = nested_function->value;
		i++;

		entry = SIMC_List_GetNext(variable->list,entry);
	}

	//Sort the table of values in the right order
	switch (function->interpolation) {
		case EVDS_VARIABLE_FUNCTION_INTERPOLATION_LINEAR:
			qsort(function->linear,function->data_count,sizeof(EVDS_VARIABLE_FVALUE_LINEAR),EVDS_InternalVariable_CompareEntries_Linear);
		break;
		case EVDS_VARIABLE_FUNCTION_INTERPOLATION_SPLINE:
			qsort(function->spline,function->data_count,sizeof(EVDS_VARIABLE_FVALUE_SPLINE),EVDS_InternalVariable_CompareEntries_Spline);
		break;
	}
	return EVDS_OK;
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Destroy function data structure
////////////////////////////////////////////////////////////////////////////////
int EVDS_InternalVariable_DestroyFunction(EVDS_VARIABLE* variable, EVDS_VARIABLE_FUNCTION* function) {
	return EVDS_OK;
}


/// Forward declarations for interpolation functions
int EVDS_InternalVariable_GetFunction_Linear(EVDS_VARIABLE_FUNCTION* function, 
											 EVDS_REAL x, EVDS_REAL y, EVDS_REAL z, EVDS_REAL* p_value);
int EVDS_InternalVariable_GetFunction_Spline(EVDS_VARIABLE_FUNCTION* function, 
											 EVDS_REAL x, EVDS_REAL y, EVDS_REAL z, EVDS_REAL* p_value);


////////////////////////////////////////////////////////////////////////////////
/// @brief Get functions value at node by index (to be called from linear interpolation function)
////////////////////////////////////////////////////////////////////////////////
int EVDS_InternalVariable_GetFunctionValue_Linear(EVDS_VARIABLE_FUNCTION* function,
														int index, EVDS_REAL y, EVDS_REAL z, EVDS_REAL* p_value) {
	if (!function->linear[index].function) { //Use the raw value from data table
		*p_value = function->linear[index].value;
	} else { //Select value from nested function and use the right interpolating function
		EVDS_VARIABLE_FUNCTION* nested_function = function->linear[index].function;
		switch (nested_function->interpolation) {
			case EVDS_VARIABLE_FUNCTION_INTERPOLATION_LINEAR:
				return EVDS_InternalVariable_GetFunction_Linear(nested_function,y,z,0.0,p_value);
			case EVDS_VARIABLE_FUNCTION_INTERPOLATION_SPLINE:
				break;
				//return EVDS_InternalVariable_GetFunction_Spline(nested_function,y,z,0.0,p_value);
		}
	}
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Get functions value at node by index (to be called from spline interpolation function)
////////////////////////////////////////////////////////////////////////////////
int EVDS_InternalVariable_GetFunctionValue_Spline(EVDS_VARIABLE_FUNCTION* function,
														int index, EVDS_REAL y, EVDS_REAL z, EVDS_REAL* p_value) {
	/*if (!function->spline[index].function) { //Use the raw value from data table
		*p_value = function->spline[index].value;
	} else { //Select value from nested function and use the right interpolating function
		EVDS_VARIABLE_FUNCTION* nested_function = function->spline[index].function;
		switch (nested_function->interpolation) {
			case EVDS_VARIABLE_FUNCTION_INTERPOLATION_LINEAR:
				return EVDS_InternalVariable_GetFunction_Linear(nested_function,y,z,0.0,p_value);
			case EVDS_VARIABLE_FUNCTION_INTERPOLATION_SPLINE:
				break;
				//return EVDS_InternalVariable_GetFunction_Spline(nested_function,y,z,0.0,p_value);
		}
	}*/
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Get value from a linear function.
///
/// If additional variables are required for nested functions, then 'y' and 'z' are used.
////////////////////////////////////////////////////////////////////////////////
int EVDS_InternalVariable_GetFunction_Linear(EVDS_VARIABLE_FUNCTION* function, 
											 EVDS_REAL x, EVDS_REAL y, EVDS_REAL z, EVDS_REAL* p_value) {
	int i;
	EVDS_REAL vi,vj,xi,xj;

	//Check for edge cases
	if (function->data_count == 0) {
		*p_value = function->constant_value;
		return EVDS_OK;
	}
	if (function->data_count == 1) {
		return EVDS_InternalVariable_GetFunctionValue_Linear(function,0,y,z,p_value);
	}
	if (x <= function->linear[0].x) {
		return EVDS_InternalVariable_GetFunctionValue_Linear(function,0,y,z,p_value);
	}
	if (x >= function->linear[function->data_count-1].x) {
		return EVDS_InternalVariable_GetFunctionValue_Linear(function,function->data_count-1,y,z,p_value);
	}

	//Find interpolation segment
	for (i = function->data_count-1; i >= 0; i--) {
		if (x > function->linear[i].x) {
			break;
		}
	}

	//Linear interpolation
	EVDS_InternalVariable_GetFunctionValue_Linear(function,i,  y,z,&vi);
	EVDS_InternalVariable_GetFunctionValue_Linear(function,i+1,y,z,&vj);
	xi = function->linear[i  ].x;
	xj = function->linear[i+1].x;

	*p_value = vi + (vj - vi)*((x - xi)/(xj - xi));
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Get value from a 1D/2D/3D function
////////////////////////////////////////////////////////////////////////////////
int EVDS_Variable_GetFunctionValue(EVDS_VARIABLE* variable, EVDS_REAL x, EVDS_REAL y, EVDS_REAL z, EVDS_REAL* p_value) {
	EVDS_VARIABLE_FUNCTION* function;
	if (!variable) return EVDS_ERROR_BAD_PARAMETER;
	if (!p_value) return EVDS_ERROR_BAD_PARAMETER;
	if ((variable->type != EVDS_VARIABLE_TYPE_FLOAT) &&
		(variable->type != EVDS_VARIABLE_TYPE_FUNCTION))return EVDS_ERROR_BAD_STATE;
#ifndef EVDS_SINGLETHREADED
	if (variable->object && variable->object->destroyed) return EVDS_ERROR_INVALID_OBJECT;
#endif

	//Float constants are accepted as zero-size tables
	if (variable->type == EVDS_VARIABLE_TYPE_FLOAT) {
		return EVDS_Variable_GetReal(variable,p_value);
	}

	//Check if the table is empty and a constant value must be used
	function = (EVDS_VARIABLE_FUNCTION*)variable->value;
	if (function->data_count == 0) {
		return EVDS_Variable_GetReal(variable,p_value);
	}

	//Select the right interpolating function
	switch (function->interpolation) {
		case EVDS_VARIABLE_FUNCTION_INTERPOLATION_LINEAR:
			return EVDS_InternalVariable_GetFunction_Linear(function,x,y,z,p_value);
		case EVDS_VARIABLE_FUNCTION_INTERPOLATION_SPLINE:
			break;
			//return EVDS_InternalVariable_GetFunction_Spline(function,x,y,z,p_value);
	}
	return EVDS_OK;
}