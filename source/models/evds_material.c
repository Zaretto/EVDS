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
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "evds.h"




////////////////////////////////////////////////////////////////////////////////
/// @brief Check if material is oxidier
///
/// @param[in] system Pointer to EVDS_SYSTEM
/// @param[in] name Null-terminated material name
///
/// @returns Error code
/// @retval EVDS_OK Successfully completed
/// @retval EVDS_ERROR_BAD_STATE Material database not loaded or material is missing
/// @retval EVDS_ERROR_INVALID_TYPE Material is not an oxidizer
////////////////////////////////////////////////////////////////////////////////
int EVDS_Material_IsOxidizer(EVDS_SYSTEM* system, const char* name) {
	EVDS_VARIABLE* database;
	EVDS_VARIABLE* material;

	if ((EVDS_System_GetDatabaseByName(system,"material",&database) == EVDS_OK) &&
		(EVDS_Variable_GetNested(database,name,&material) == EVDS_OK)) {
		char class_str[256] = { 0 };
		EVDS_VARIABLE* attribute;
		if (EVDS_Variable_GetAttribute(material,"class",&attribute) == EVDS_OK) {
			EVDS_Variable_GetString(attribute,class_str,255,0);
			if (strcmp(class_str,"oxidizer") == 0) {
				return EVDS_OK;
			} else {
				return EVDS_ERROR_INVALID_TYPE;
			}
		}
	}
	return EVDS_ERROR_BAD_STATE;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Check if material is fuel
///
/// @param[in] system Pointer to EVDS_SYSTEM
/// @param[in] name Null-terminated material name
///
/// @returns Error code
/// @retval EVDS_OK Successfully completed
/// @retval EVDS_ERROR_BAD_STATE Material database not loaded or material is missing
/// @retval EVDS_ERROR_INVALID_TYPE Material is not an oxidizer
////////////////////////////////////////////////////////////////////////////////
int EVDS_Material_IsFuel(EVDS_SYSTEM* system, const char* name) {
	EVDS_VARIABLE* database;
	EVDS_VARIABLE* material;

	if ((EVDS_System_GetDatabaseByName(system,"material",&database) == EVDS_OK) &&
		(EVDS_Variable_GetNested(database,name,&material) == EVDS_OK)) {
		char class_str[256] = { 0 };
		EVDS_VARIABLE* attribute;
		if (EVDS_Variable_GetAttribute(material,"class",&attribute) == EVDS_OK) {
			EVDS_Variable_GetString(attribute,class_str,255,0);
			if (strcmp(class_str,"fuel") == 0) {
				return EVDS_OK;
			} else {
				return EVDS_ERROR_INVALID_TYPE;
			}
		}
	}
	return EVDS_ERROR_BAD_STATE;
}
