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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "evds.h"
#include "sim_xml.h"


////////////////////////////////////////////////////////////////////////////////
/// @brief Write a single variable
////////////////////////////////////////////////////////////////////////////////
int EVDS_Internal_SaveVariable(EVDS_VARIABLE* variable, SIMC_XML_DOCUMENT* doc, 
							   SIMC_XML_ELEMENT* root, EVDS_VARIABLE* parent, int is_attribute) {
	SIMC_XML_ELEMENT* element;
	SIMC_LIST_ENTRY* entry;
	EVDS_VARIABLE* child;
	char* buffer = alloca(1024*sizeof(char));
	memset(buffer,0,1024*sizeof(char)); //Make sure buffer is empty beforehand

	//Special case: do not write empty "comments" variables
	if (strncmp(variable->name,"comments",64) == 0) {
		char first_char = 0;
		EVDS_Variable_GetString(variable,&first_char,1,0);
		if (!first_char) return EVDS_OK;
	}
	//Special case: do not write empty "geometry.cross_sections" variables
	if ((strncmp(variable->name,"geometry.cross_sections",64) == 0) &&
		(variable->type == EVDS_VARIABLE_TYPE_NESTED)) {
		int count = 0;
		if (variable->list) { //Count number of elements inside
			entry = SIMC_List_GetFirst(variable->list);
			while (entry) {
				count++;
				entry = SIMC_List_GetNext(variable->list,entry);
			}
		}
		if (count <= 1) return EVDS_OK;
	}

	//Create entry for this object
	if (!is_attribute) {
		if (parent) {
			SIMC_XML_AddElement(doc,root,&element,variable->name);
		} else {
			SIMC_XML_AddElement(doc,root,&element,"parameter");
			SIMC_XML_AddAttribute(doc,element,"name",variable->name);
		}
	}

	//Save value
	if (variable->type == EVDS_VARIABLE_TYPE_STRING) {
		int buffer_length;
		EVDS_Variable_GetString(variable,0,0,&buffer_length); //Get string length
		buffer = alloca((buffer_length+1)*sizeof(char)); //Allocate new big buffer
		EVDS_Variable_GetString(variable,buffer,buffer_length,0); //Read string there
		buffer[buffer_length] = 0; //Null-terminate
	} else if (variable->type == EVDS_VARIABLE_TYPE_FLOAT) {
		EVDS_REAL value;
		EVDS_Variable_GetReal(variable,&value);

		//Replace near-zero values with zero values
		if ((value <= EVDS_EPS) && (value >= -EVDS_EPS)) value = 0.0;

		//Only write non-null values, unless it's a parameter
		if ((value != 0.0) || (!parent)) {
			snprintf(buffer,1023,"%.15g",value);
		}
		//EVDS_Variable_SetReal(variable,real_value);
	} else if (variable->type == EVDS_VARIABLE_TYPE_VECTOR) {
		EVDS_VECTOR value;
		EVDS_Variable_GetVector(variable,&value);
		
		//Replace near-zero values with zero values
		if ((value.x <= EVDS_EPS) && (value.x >= -EVDS_EPS)) value.x = 0.0;
		if ((value.y <= EVDS_EPS) && (value.y >= -EVDS_EPS)) value.y = 0.0;
		if ((value.z <= EVDS_EPS) && (value.z >= -EVDS_EPS)) value.z = 0.0;

		snprintf(buffer,1023,"%.15g %.15g %.15g",value.x,value.y,value.z);
	} else if (variable->type == EVDS_VARIABLE_TYPE_QUATERNION) {
		EVDS_QUATERNION value;
		EVDS_Variable_GetQuaternion(variable,&value);

		//Replace near-zero values with zero values
		if ((value.q[0] <= EVDS_EPS) && (value.q[0] >= -EVDS_EPS)) value.q[0] = 0.0;
		if ((value.q[1] <= EVDS_EPS) && (value.q[1] >= -EVDS_EPS)) value.q[1] = 0.0;
		if ((value.q[2] <= EVDS_EPS) && (value.q[2] >= -EVDS_EPS)) value.q[2] = 0.0;
		if ((value.q[3] <= EVDS_EPS) && (value.q[3] >= -EVDS_EPS)) value.q[3] = 0.0;

		snprintf(buffer,1023,"%.15g %.15g %.15g %.15g",value.q[0],value.q[1],value.q[2],value.q[3]);
	}

	//Add text value if not tested
	if (variable->type != EVDS_VARIABLE_TYPE_NESTED) {
		if (!is_attribute) {
			SIMC_XML_SetText(doc,element,buffer); //FIXME: does not support null characters in buffer

			//Check if variable value is like a float/quaternion/vector
			if (variable->type == EVDS_VARIABLE_TYPE_STRING) {
				char *end, *end_x, *end_y, *end_z, *end_w;
				end = buffer+strlen(buffer); //FIXME
	
				strtod(buffer,&end_x);
				strtod(end_x,&end_y);
				strtod(end_y,&end_z);
				strtod(end_z,&end_w);
				if ((end_x == end) || (end_y == end) ||
					(end_z == end) || (end_w == end)) {
					SIMC_XML_AddAttribute(doc,element,"type","string");
				}
			}
		} else {
			SIMC_XML_AddAttribute(doc,root,variable->name,buffer);
		}
	}

	if (!is_attribute) {
		//Save all attributes
		if (variable->attributes) {
			entry = SIMC_List_GetFirst(variable->attributes);
			while (entry) {
				child = (EVDS_VARIABLE*)SIMC_List_GetData(variable->attributes,entry);
				EVDS_ERRCHECK(EVDS_Internal_SaveVariable(child,doc,element,variable,1));
				entry = SIMC_List_GetNext(variable->attributes,entry);
			}
		}

		//Save all children variables
		if (variable->list) {
			entry = SIMC_List_GetFirst(variable->list);
			while (entry) {
				child = (EVDS_VARIABLE*)SIMC_List_GetData(variable->list,entry);
				EVDS_ERRCHECK(EVDS_Internal_SaveVariable(child,doc,element,variable,0));
				entry = SIMC_List_GetNext(variable->list,entry);
			}
		}
	}

	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Write a single object
////////////////////////////////////////////////////////////////////////////////
int EVDS_Internal_SaveObject(EVDS_OBJECT* object, SIMC_XML_DOCUMENT* doc,
							 SIMC_XML_ELEMENT* root, EVDS_OBJECT_SAVEEX* info) {
	SIMC_XML_ELEMENT* element;
	SIMC_LIST_ENTRY* entry;
	EVDS_OBJECT* child;
	EVDS_VARIABLE* child_variable;
	EVDS_STATE_VECTOR vector;
	double pitch,yaw,roll;

	//Skip objects generated by modifier
	//if (object->modifier && (!(info->flags & EVDS_OBJECT_SAVEEX_SAVE_COPIES))) return EVDS_OK;

	//Do not save object itself if applies
	if (info && (info->flags & EVDS_OBJECT_SAVEEX_ONLY_CHILDREN)) goto skip_object;

	//Create entry for this object
	EVDS_ERRCHECK(SIMC_XML_AddElement(doc,root,&element,"object"));
	SIMC_SRW_EnterRead(object->name_lock);
		EVDS_ERRCHECK(SIMC_XML_AddAttribute(doc,element,"name",object->name));
	SIMC_SRW_LeaveRead(object->name_lock);
	SIMC_SRW_EnterRead(object->type_lock);
		EVDS_ERRCHECK(SIMC_XML_AddAttribute(doc,element,"type",object->type));
	SIMC_SRW_LeaveRead(object->type_lock);
	if (info && (info->flags & EVDS_OBJECT_SAVEEX_SAVE_UIDS)) {
		EVDS_ERRCHECK(SIMC_XML_AddAttributeDouble(doc,element,"uid",object->uid));
	}

	//Write position/orientation
	EVDS_Object_GetStateVector(object,&vector);
	EVDS_Quaternion_ToEuler(&vector.orientation,vector.orientation.coordinate_system,&roll,&pitch,&yaw);
	EVDS_ERRCHECK(SIMC_XML_AddAttributeDouble(doc,element,"x",vector.position.x));
	EVDS_ERRCHECK(SIMC_XML_AddAttributeDouble(doc,element,"y",vector.position.y));
	EVDS_ERRCHECK(SIMC_XML_AddAttributeDouble(doc,element,"z",vector.position.z));
	EVDS_ERRCHECK(SIMC_XML_AddAttributeDouble(doc,element,"vx",vector.velocity.x));
	EVDS_ERRCHECK(SIMC_XML_AddAttributeDouble(doc,element,"vy",vector.velocity.y));
	EVDS_ERRCHECK(SIMC_XML_AddAttributeDouble(doc,element,"vz",vector.velocity.z));
	EVDS_ERRCHECK(SIMC_XML_AddAttributeDouble(doc,element,"pitch",EVDS_DEG(pitch)));
	EVDS_ERRCHECK(SIMC_XML_AddAttributeDouble(doc,element,"yaw",EVDS_DEG(yaw)));
	EVDS_ERRCHECK(SIMC_XML_AddAttributeDouble(doc,element,"roll",EVDS_DEG(roll)));
	if (info && (info->flags & EVDS_OBJECT_SAVEEX_SAVE_FULL_STATE)) {
		EVDS_ERRCHECK(SIMC_XML_AddAttributeDouble(doc,element,"time",vector.time));
		EVDS_ERRCHECK(SIMC_XML_AddAttributeDouble(doc,element,"q0",vector.orientation.q[0]));
		EVDS_ERRCHECK(SIMC_XML_AddAttributeDouble(doc,element,"q1",vector.orientation.q[1]));
		EVDS_ERRCHECK(SIMC_XML_AddAttributeDouble(doc,element,"q2",vector.orientation.q[2]));
		EVDS_ERRCHECK(SIMC_XML_AddAttributeDouble(doc,element,"q3",vector.orientation.q[3]));
		EVDS_ERRCHECK(SIMC_XML_AddAttributeDouble(doc,element,"ax",vector.acceleration.x));
		EVDS_ERRCHECK(SIMC_XML_AddAttributeDouble(doc,element,"ay",vector.acceleration.y));
		EVDS_ERRCHECK(SIMC_XML_AddAttributeDouble(doc,element,"az",vector.acceleration.z));
		EVDS_ERRCHECK(SIMC_XML_AddAttributeDouble(doc,element,"ang_ax",vector.angular_acceleration.x));
		EVDS_ERRCHECK(SIMC_XML_AddAttributeDouble(doc,element,"ang_ay",vector.angular_acceleration.y));
		EVDS_ERRCHECK(SIMC_XML_AddAttributeDouble(doc,element,"ang_az",vector.angular_acceleration.z));
		EVDS_ERRCHECK(SIMC_XML_AddAttributeDouble(doc,element,"ang_vx",vector.angular_velocity.x));
		EVDS_ERRCHECK(SIMC_XML_AddAttributeDouble(doc,element,"ang_vy",vector.angular_velocity.y));
		EVDS_ERRCHECK(SIMC_XML_AddAttributeDouble(doc,element,"ang_vz",vector.angular_velocity.z));
	}

	//Add all variables
	entry = SIMC_List_GetFirst(object->variables);
	while (entry) {
		child_variable = (EVDS_VARIABLE*)SIMC_List_GetData(object->variables,entry);
		EVDS_ERRCHECK(EVDS_Internal_SaveVariable(child_variable,doc,element,0,0));
		entry = SIMC_List_GetNext(object->variables,entry);
	}

skip_object:
	if (info && (info->flags & EVDS_OBJECT_SAVEEX_ONLY_CHILDREN)) {
		info->flags = info->flags & (~EVDS_OBJECT_SAVEEX_ONLY_CHILDREN);
		element = root;
	}

	//Create all children objects
	entry = SIMC_List_GetFirst(object->raw_children);
	while (entry) {
		child = (EVDS_OBJECT*)SIMC_List_GetData(object->raw_children,entry);
		EVDS_ERRCHECK(EVDS_Internal_SaveObject(child,doc,element,info));
		entry = SIMC_List_GetNext(object->raw_children,entry);
	}

	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Save object and its children to file
////////////////////////////////////////////////////////////////////////////////
int EVDS_Object_SaveToFile(EVDS_OBJECT* object, const char* filename) {
	SIMC_XML_DOCUMENT* doc;
	SIMC_XML_ELEMENT* root;
	if (!object) return EVDS_ERROR_BAD_PARAMETER;
	if (!filename) return EVDS_ERROR_BAD_PARAMETER;

	EVDS_ERRCHECK(SIMC_XML_Create(&doc));
	EVDS_ERRCHECK(SIMC_XML_AddRootElement(doc,&root,"EVDS"));
	EVDS_ERRCHECK(SIMC_XML_AddAttributeDouble(doc,root,"version",EVDS_VERSION));
	EVDS_ERRCHECK(EVDS_Internal_SaveObject(object,doc,root,0));
	EVDS_ERRCHECK(SIMC_XML_Save(doc,filename));
	EVDS_ERRCHECK(SIMC_XML_Close(doc));
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Save object and return its description
////////////////////////////////////////////////////////////////////////////////
int EVDS_Object_SaveToString(EVDS_OBJECT* object, char** description) {
	SIMC_XML_DOCUMENT* doc;
	SIMC_XML_ELEMENT* root;
	if (!object) return EVDS_ERROR_BAD_PARAMETER;
	if (!description) return EVDS_ERROR_BAD_PARAMETER;

	EVDS_ERRCHECK(SIMC_XML_Create(&doc));
	EVDS_ERRCHECK(SIMC_XML_AddRootElement(doc,&root,"EVDS"));
	EVDS_ERRCHECK(SIMC_XML_AddAttributeDouble(doc,root,"version",EVDS_VERSION));
	EVDS_ERRCHECK(EVDS_Internal_SaveObject(object,doc,root,0));
	EVDS_ERRCHECK(SIMC_XML_SaveString(doc,description));
	EVDS_ERRCHECK(SIMC_XML_Close(doc));
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Empty data for EVDS_Object_SaveEx()
////////////////////////////////////////////////////////////////////////////////
EVDS_OBJECT_SAVEEX EVDS_Internal_SaveEx = { 0 };


////////////////////////////////////////////////////////////////////////////////
/// @brief Save object
////////////////////////////////////////////////////////////////////////////////
int EVDS_Object_SaveEx(EVDS_OBJECT* object, const char* filename, EVDS_OBJECT_SAVEEX* info) {
	SIMC_XML_DOCUMENT* doc;
	SIMC_XML_ELEMENT* root;
	if (!object) return EVDS_ERROR_BAD_PARAMETER;
	if (!info) info = &EVDS_Internal_SaveEx;
	if ((!info) && (!filename)) return EVDS_ERROR_BAD_PARAMETER;

	EVDS_ERRCHECK(SIMC_XML_Create(&doc));
	EVDS_ERRCHECK(SIMC_XML_AddRootElement(doc,&root,"EVDS"));
	EVDS_ERRCHECK(SIMC_XML_AddAttributeDouble(doc,root,"version",EVDS_VERSION));
	EVDS_ERRCHECK(EVDS_Internal_SaveObject(object,doc,root,info));
	if (!filename) {
		EVDS_ERRCHECK(SIMC_XML_SaveString(doc,&info->description));
	} else {
		EVDS_ERRCHECK(SIMC_XML_Save(doc,filename));
	}
	EVDS_ERRCHECK(SIMC_XML_Close(doc));
	return EVDS_OK;
}
