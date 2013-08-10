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
/// @brief Initialize function data structure
////////////////////////////////////////////////////////////////////////////////
int EVDS_InternalVariable_InitializeFunction(EVDS_VARIABLE* variable, EVDS_VARIABLE_FUNCTION* function) {
	return EVDS_OK;
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Destroy function data structure
////////////////////////////////////////////////////////////////////////////////
int EVDS_InternalVariable_DestroyFunction(EVDS_VARIABLE* variable, EVDS_VARIABLE_FUNCTION* function) {
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Get value from a 1D function 
////////////////////////////////////////////////////////////////////////////////
int EVDS_Variable_GetFunction1D(EVDS_VARIABLE* variable, EVDS_REAL x, EVDS_REAL* p_value) {
	/*int i;
	EVDS_VARIABLE_FUNCTION* table;
	if (!variable) return EVDS_ERROR_BAD_PARAMETER;
	if (!p_value) return EVDS_ERROR_BAD_PARAMETER;
	if ((variable->type != EVDS_VARIABLE_TYPE_FLOAT) &&
		(variable->type != EVDS_VARIABLE_TYPE_FUNCTION))return EVDS_ERROR_BAD_STATE;
#ifndef EVDS_SINGLETHREADED
	if (variable->object && variable->object->destroyed) return EVDS_ERROR_INVALID_OBJECT;
#endif

	//Float constants are accepted as zero-size tables
	if (variable->type == EVDS_VARIABLE_TYPE_FLOAT) {
		*p_value = *((double*)variable->value);
		return EVDS_OK;
	}

	//Get table and check if a lesser dimension function must be used
	table = (EVDS_VARIABLE_FUNCTION*)variable->value;
	if (table->data1d_count == 0) {
		return EVDS_Variable_GetReal(variable,p_value);
	}

	//Check for edge cases
	if (table->data1d_count == 1) {
		*p_value = table->data1d[0].f;
		return EVDS_OK;
	}
	if (x <= table->data1d[0].x) {
		*p_value = table->data1d[0].f;
		return EVDS_OK;
	}
	if (x >= table->data1d[table->data1d_count-1].x) {
		*p_value = table->data1d[table->data1d_count-1].f;
		return EVDS_OK;
	}

	//Find interpolation segment
	for (i = table->data1d_count-1; i >= 0; i--) {
		if (x > table->data1d[i].x) {
			break;
		}
	}

	//Linear interpolation
#if (defined(_MSC_VER) && (_MSC_VER >= 1500) && (_MSC_VER < 1600))
	{
		double A = (table->data1d[i+1].x - table->data1d[i].x);
		double B = (x - table->data1d[i].x) / A;
		*p_value = table->data1d[i].f  + (table->data1d[i+1].f - table->data1d[i].f) * B;

		//*p_value = table->data[i].f  + (table->data[i+1].f - table->data[i].f) * 
			//((x - table->data[i].x) / A);
		//*p_value = table->data[i].f  + (table->data[i+1].f - table->data[i].f) * 
			//((x - table->data[i].x) / (table->data[i+1].x - table->data[i].x));
	}
#else
	*p_value = table->data1d[i].f  + (table->data1d[i+1].f - table->data1d[i].f) *
		((x - table->data1d[i].x) / (table->data1d[i+1].x - table->data1d[i].x));
#endif*/
	return EVDS_OK;
}
