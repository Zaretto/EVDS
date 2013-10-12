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
#include <stdio.h>
#include <string.h>
#include "evds.h"




////////////////////////////////////////////////////////////////////////////////
/// @brief Return version information.
///
/// The string format is "Xy", where X is the API update number, and y is the internal code update
/// number (within a single API update, expressed as a letter of the alphabet).
///
/// The integer version is expressed as "X*100+Y" where X is the API update number, and y is the
/// internal code update number.
///
/// Both parameters are optional and can can be null. If requested, the version string
/// will not exceed 64 bytes in length.
///
/// Two libraries will only be compatible when their library API versions match up.
/// Change in API version number indicates new API was added or old API was removed,
/// while change in internal code update means some code inside the library itself
/// was modified.
///
/// Example of use:
/// ~~~{.c}
///		int version;
///		char version_string[64];
///		EVDS_Version(&version,version_string);
///		if (version < 101) { //Check for version "1a"
///			//Incompatible version
///			return;
///		}
///		//Version string equal to "1a"
/// ~~~
///
/// @param[out] version If not null, library version is written here
/// @param[out] version_string If not null, library version as string is copied here
///
/// @returns Always returns EVDS_OK 
////////////////////////////////////////////////////////////////////////////////
int EVDS_Version(int* version, char* version_string) {
	if (version) *version = EVDS_VERSION;
	if (version_string) {
		int major_version = EVDS_VERSION/100;
		int minor_version = (EVDS_VERSION % 100)-1;
		if (minor_version < 26) {
			snprintf(version_string,64,"%d%c",major_version,minor_version+'a');
		} else {
			snprintf(version_string,64,"%d%c%c",major_version,((minor_version/26)-1)+'a',(minor_version % 26)+'a');
		}
	}
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// Signals an assert has failed
////////////////////////////////////////////////////////////////////////////////
int EVDS_AssertFailed(const char* what, const char* filename, int line) {
	printf("Assert failed: %s (%s:%d)\n", what, filename, line);
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Table of units of measurement and conversion factors (to SI)
////////////////////////////////////////////////////////////////////////////////
const struct {
	char* name;
	double scale_factor;
	double offset_factor;
} EVDS_Internal_UnitsTable[] = {
	//SI units
	{ "m",		1.0 },
	{ "kg",		1.0 },
	{ "K",		1.0 },
	{ "W",		1.0 },
	//Common metric units
	{ "C",		1.0, 273.15 },
	//Impertial/british units
	{ "ft",		0.3048 },
	{ "lb",		0.453592 },
	{ "lbs",	0.453592 },
	{ "R",		5.0/9.0 },
	{ "btu",	1054.35026444 },
	//Temp hack
	{ "kg/m3",	1.0 },
	{ "lb/ft3", 16.0184634 },
	{ "btu/(lb R)", 1054.35026444/(0.45359237*5.0/9.0) },
	{ "btu/(ft s R)", 1054.35026444/(0.3048*5.0/9.0) },
};
const int EVDS_Internal_UnitsTableCount = 
	sizeof(EVDS_Internal_UnitsTable) / sizeof(EVDS_Internal_UnitsTable[0]);


////////////////////////////////////////////////////////////////////////////////
/// @brief Convert a string to EVDS_REAL, accounting for units of measurement, returning EVDS_REAL in metric units.
///
/// @param[in] str Pointer to input string
/// @param[out] str_end Pointer to what follows after the real number in the input string (can be null)
/// @param[out] p_value The read value will be written here
///
/// @returns Error code
/// @retval EVDS_OK Successfully completed
/// @retval EVDS_ERROR_BAD_PARAMETER "str" is null
/// @retval EVDS_ERROR_BAD_PARAMETER "p_value" is null
/// @retval EVDS_ERROR_SYNTAX Could not parse string in its entirety as an EVDS_REAL
////////////////////////////////////////////////////////////////////////////////
int EVDS_StringToReal(const char* str, char** str_end, EVDS_REAL* p_value) {
	int i;
	char* end;
	EVDS_REAL value;
	if (!str) return EVDS_ERROR_BAD_PARAMETER;
	if (!p_value) return EVDS_ERROR_BAD_PARAMETER;

	//Skip whitespace
	while (*str == ' ') str++;

	//Get value itself
	value = strtod(str,&end);
	if (str_end) *str_end = end;

	//Check if EPS must be added or subtracted
	if (*end == '+') { value += value*EVDS_EPS; end++; }
	if (*end == '-') { value -= value*EVDS_EPS; end++; }

	//Check if units of measurements can be parsed
	while (*end == ' ') end++;
	for (i = 0; i < EVDS_Internal_UnitsTableCount; i++) {
		if (strcmp(end,EVDS_Internal_UnitsTable[i].name) == 0) {
			value *= EVDS_Internal_UnitsTable[i].scale_factor;
			value += EVDS_Internal_UnitsTable[i].offset_factor;
			end += strlen(EVDS_Internal_UnitsTable[i].name);
			break;
		}
	}

	//Return and check if entire input string was parsed
	*p_value = value;
	if (str_end) *str_end = end;
	if (!(*end)) return EVDS_OK;

	//Something was left in the string
	return EVDS_ERROR_SYNTAX;
}