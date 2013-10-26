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
#include <math.h>

#include "evds.h"
#include "sim_xml.h"


////////////////////////////////////////////////////////////////////////////////
/// Table of variables that must be renamed (for backwards compatibility)
////////////////////////////////////////////////////////////////////////////////
struct {
	int last_version;		/// Remapping only for files older or matching this version
	char* object_type;		/// Object type
	char* old_name;			/// Old variable name
	char* new_name;			/// New variable name (in most new EVDS version)
} EVDS_Internal_ParameterRemappingTable[] = {
	{ 31, "fuel_tank",		"fuel_type",			"fuel.type" },
	{ 31, "fuel_tank",		"fuel_mass",			"fuel.mass" },
	{ 31, "fuel_tank",		"fuel_volume",			"fuel.volume" },
	{ 31, "fuel_tank",		"fuel_capacity",		"fuel.capacity" },
	{ 31, "fuel_tank",		"is_cryogenic",			"fuel.is_cryogenic" },
	{ 31, "fuel_tank",		"load_ratio",			"fuel.mass" },
	{ 31, "fuel_tank",		"load_ratio_percent",	"fuel.load_ratio_percent" },
	{ 31, "fuel_tank",		"upper_radius",			"geometry.upper_radius" },
	{ 31, "fuel_tank",		"lower_radius",			"geometry.lower_radius" },
	{ 31, "fuel_tank",		"outer_radius",			"geometry.outer_radius" },
	{ 31, "fuel_tank",		"inner_radius",			"geometry.inner_radius" },
	{ 31, "fuel_tank",		"middle_length",		"geometry.middle_length" },
};
const int EVDS_Internal_ParameterRemappingTableCount = 
	sizeof(EVDS_Internal_ParameterRemappingTable) / sizeof(EVDS_Internal_ParameterRemappingTable[0]);


////////////////////////////////////////////////////////////////////////////////
/// Table of object names that must be renamed (for backwards compatibility)
////////////////////////////////////////////////////////////////////////////////
struct {
	int last_version;		/// Remapping only for files older or matching this version
	char* old_name;			/// Old object name
	char* new_name;			/// New object name (in most new EVDS version)
} EVDS_Internal_ObjectRemappingTable[] = {
	{ 0, "test", "misc.test" },
};
const int EVDS_Internal_ObjectRemappingTableCount = 
	sizeof(EVDS_Internal_ObjectRemappingTable) / sizeof(EVDS_Internal_ObjectRemappingTable[0]);




////////////////////////////////////////////////////////////////////////////////
/// @brief Load parameter from the XML file
///
/// @param[in] nested_in_function All variables nested inside function are treated as functions.
////////////////////////////////////////////////////////////////////////////////
int EVDS_Internal_LoadParameter(EVDS_OBJECT* object, EVDS_VARIABLE* parent_variable, 
								SIMC_XML_DOCUMENT* doc, SIMC_XML_ELEMENT* element, SIMC_XML_ATTRIBUTE* attribute,
								EVDS_OBJECT_LOADEX* info, int nested_in_function) {
	int i;
	char* value;
	char* name;
	char* vector_type;
	char* variable_type;
	double real_value;
	double x,y,z,w;
	EVDS_VARIABLE* variable;
	EVDS_VARIABLE_TYPE type = EVDS_VARIABLE_TYPE_NESTED; //Nested unless specified otherwise
	SIMC_XML_ELEMENT* nested_element;
	SIMC_XML_ATTRIBUTE* nested_attribute;


	//Get name, type and text (value) of the parameter (variable, attribute)
	if (element) {
		//Parameter/variable
		EVDS_ERRCHECK(SIMC_XML_GetAttribute(doc,element,"name",&name));
		EVDS_ERRCHECK(SIMC_XML_GetText(doc,element,&value));

		//If variable is nested, use element name for variable name
		if ((name[0] == '\0') && (parent_variable)) {
			EVDS_ERRCHECK(SIMC_XML_GetName(doc,element,&name));
		}

		//Get variable type
		EVDS_ERRCHECK(SIMC_XML_GetAttribute(doc,element,"type",&variable_type));
		if (!variable_type) variable_type = "";

		//Get vector type
		EVDS_ERRCHECK(SIMC_XML_GetAttribute(doc,element,"vector_type",&vector_type));
		if (!vector_type) vector_type = "";
	} else {
		//Attribute
		EVDS_ERRCHECK(SIMC_XML_GetAttributeName(doc,attribute,&name));
		EVDS_ERRCHECK(SIMC_XML_GetAttributeText(doc,attribute,&value));
		vector_type = "";
		variable_type = "";
	}
	if (!name) name = "";


	//Remap parameter name (to provide compatibility with old file versions)
	for (i = 0; i < EVDS_Internal_ParameterRemappingTableCount; i++) {
		//Find object relevant to the variable
		EVDS_OBJECT* related_object = object;
		if ((!related_object) && (parent_variable)) related_object = parent_variable->object;

		//Check if remapping criteria are satisfied
		if ((related_object) &&
			((info->version <= EVDS_Internal_ParameterRemappingTable[i].last_version) ||
			 (EVDS_Internal_ParameterRemappingTable[i].last_version == 0)) &&
			(strncmp(related_object->type,EVDS_Internal_ParameterRemappingTable[i].object_type,256) == 0) &&
			(strcmp(name,EVDS_Internal_ParameterRemappingTable[i].old_name) == 0)) {
			//Use new name if object type matches, and old parameter name is used
			name = EVDS_Internal_ParameterRemappingTable[i].new_name;
			break;
		}
	}


	//Try to guess data type based on value (if type is not defined)
	if (value && (!variable_type[0])) {
		char *end_ptr, *end_value;
		EVDS_StringToReal(value,&end_ptr,&real_value); //Convert string to a real value
		end_value = value + strlen(value);

		if (end_ptr == end_value) { //All string consumed: single variable
			type = EVDS_VARIABLE_TYPE_FLOAT;
		} else { //Multiple variables, possibly a vector
			char *end_x, *end_y, *end_z, *end_w;
			EVDS_StringToReal(value,&end_x,&x);
			EVDS_StringToReal(end_x,&end_y,&y);
			EVDS_StringToReal(end_y,&end_z,&z);
			EVDS_StringToReal(end_z,&end_w,&w);

			if ((end_x != end_value) &&
				(end_y != end_value) &&
				(end_z == end_value) &&
				(end_w == end_value)) { //Three floats make up a vector
				type = EVDS_VARIABLE_TYPE_VECTOR;
			} else if ((end_x != end_value) &&
				(end_y != end_value) &&
				(end_z != end_value) &&
				(end_w == end_value)) { //Four floats make up a quaternion
				type = EVDS_VARIABLE_TYPE_QUATERNION;
			} else if (value[0]) { //Not null and does not look like a vector
				type = EVDS_VARIABLE_TYPE_STRING; //May be string or a nested variable
			}
		}
	} else {
		if (variable_type[0]) { //Override automatically detected type
			if (strcmp(variable_type,"function") == 0) {
				type = EVDS_VARIABLE_TYPE_FUNCTION;
			} else if (strcmp(variable_type,"string") == 0) {
				type = EVDS_VARIABLE_TYPE_STRING;
			} if (strcmp(variable_type,"float") == 0) {
				type = EVDS_VARIABLE_TYPE_FLOAT;
			} if (strcmp(variable_type,"vector") == 0) {
				type = EVDS_VARIABLE_TYPE_VECTOR;
			} if (strcmp(variable_type,"quaternion") == 0) {
				type = EVDS_VARIABLE_TYPE_QUATERNION;
			}
		}
	}
	

	//Check if the variable is a nested or a function anyway (can't be overriden)
	if (element && (!nested_in_function)) {
		//If any nested tag is present, it's a nested variable (possibly with text)
		EVDS_ERRCHECK(SIMC_XML_GetElement(doc,element,&nested_element,0));
		if (nested_element) type = EVDS_VARIABLE_TYPE_NESTED;

		//If 'data' nested tag present, it's a function
		EVDS_ERRCHECK(SIMC_XML_GetElement(doc,element,&nested_element,"data"));
		if (nested_element) type = EVDS_VARIABLE_TYPE_FUNCTION;
	}

	//If this variable is nested inside a function variable and it's called 'data', 
	// it has always "function" type (so the functions are initialized recursively)
	if (nested_in_function) {
		if (strcmp(name,"data") == 0) {
			type = EVDS_VARIABLE_TYPE_FUNCTION;
		}
	}


	//Add variable to object or to parent variable
	if (object) {
		EVDS_ERRCHECK(EVDS_Object_AddVariable(object,name,type,&variable));
	} else {
		if (element) {
			EVDS_ERRCHECK(EVDS_Variable_AddNested(parent_variable,name,type,&variable));
		} else {
			EVDS_ERRCHECK(EVDS_Variable_AddAttribute(parent_variable,name,type,&variable));
		}
	}
	
	//Store data into variable
	if (type == EVDS_VARIABLE_TYPE_STRING) {
		if (!value) value = ""; //If no value defined, give it an empty value
		EVDS_ERRCHECK(EVDS_Variable_SetString(variable,value,strlen(value)));
	} else if (type == EVDS_VARIABLE_TYPE_FLOAT) {
		EVDS_ERRCHECK(EVDS_Variable_SetReal(variable,real_value));
	} else if (type == EVDS_VARIABLE_TYPE_VECTOR) {
		EVDS_VECTOR vector;
		vector.x = x;
		vector.y = y;
		vector.z = z;
		vector.coordinate_system = object;
		vector.pcoordinate_system = 0;
		vector.vcoordinate_system = 0;
		vector.derivative_level = EVDS_VECTOR_POSITION;

		if (strcmp(vector_type,"velocity") == 0)				vector.derivative_level = EVDS_VECTOR_VELOCITY;
		if (strcmp(vector_type,"acceleration") == 0)			vector.derivative_level = EVDS_VECTOR_ACCELERATION;
		if (strcmp(vector_type,"angular_velocity") == 0)		vector.derivative_level = EVDS_VECTOR_ANGULAR_VELOCITY;
		if (strcmp(vector_type,"angular_acceleration") == 0)	vector.derivative_level = EVDS_VECTOR_ANGULAR_ACCELERATION;
		if (strcmp(vector_type,"force") == 0)					vector.derivative_level = EVDS_VECTOR_FORCE;
		if (strcmp(vector_type,"torque") == 0)					vector.derivative_level = EVDS_VECTOR_TORQUE;
		
		EVDS_ERRCHECK(EVDS_Variable_SetVector(variable,&vector));
	} else if (type == EVDS_VARIABLE_TYPE_QUATERNION) {
		EVDS_QUATERNION q;
		q.q[0] = x;
		q.q[1] = y;
		q.q[2] = z;
		q.q[3] = w;
		q.coordinate_system = object;
		EVDS_ERRCHECK(EVDS_Variable_SetQuaternion(variable,&q));
	} else if (type == EVDS_VARIABLE_TYPE_FUNCTION) {
		EVDS_VARIABLE_FUNCTION* function = (EVDS_VARIABLE_FUNCTION*)variable->value;

		//Read all nested data entries
		EVDS_ERRCHECK(SIMC_XML_GetElement(doc,element,&nested_element,0));
		while (nested_element) {
			EVDS_ERRCHECK(EVDS_Internal_LoadParameter(0,variable,doc,nested_element,0,info,1));
			EVDS_ERRCHECK(SIMC_XML_Iterate(doc,element,&nested_element,0));
		}

		//Load all nested attributes
		EVDS_ERRCHECK(SIMC_XML_GetFirstAttribute(doc,element,&nested_attribute));
		while (nested_attribute) {
			EVDS_ERRCHECK(EVDS_Internal_LoadParameter(0,variable,doc,0,nested_attribute,info,1));
			EVDS_ERRCHECK(SIMC_XML_IterateAttributes(doc,nested_attribute,&nested_attribute));
		}

		//Initialize function table
		function->constant_value = 0.0;
		EVDS_ERRCHECK(EVDS_InternalVariable_InitializeFunction(variable,function,value));
	} else if (element) {
		//Save text value of the nested element
		if (value) {
			EVDS_ERRCHECK(EVDS_Variable_SetString(variable,value,strlen(value)));
		}

		//Load all nested parameters
		EVDS_ERRCHECK(SIMC_XML_GetElement(doc,element,&nested_element,0));
		while (nested_element) {
			EVDS_ERRCHECK(EVDS_Internal_LoadParameter(0,variable,doc,nested_element,0,info,nested_in_function));
			EVDS_ERRCHECK(SIMC_XML_Iterate(doc,element,&nested_element,0));
		}

		//Load all nested attributes
		EVDS_ERRCHECK(SIMC_XML_GetFirstAttribute(doc,element,&nested_attribute));
		while (nested_attribute) {
			EVDS_ERRCHECK(EVDS_Internal_LoadParameter(0,variable,doc,0,nested_attribute,info,nested_in_function));
			EVDS_ERRCHECK(SIMC_XML_IterateAttributes(doc,nested_attribute,&nested_attribute));
		}
	}
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Load object from the XML file
////////////////////////////////////////////////////////////////////////////////
int EVDS_Internal_LoadObject(EVDS_OBJECT* parent, SIMC_XML_DOCUMENT* doc, SIMC_XML_ELEMENT* root, EVDS_OBJECT** p_object,
							 EVDS_OBJECT_LOADEX* info) {
	EVDS_OBJECT* object;
	SIMC_XML_ELEMENT* element;
	char *name, *type;
	double x,y,z,vx,vy,vz,pitch,yaw,roll;
	double q0,q1,q2,q3;//,time;
	double ax,ay,az,ang_ax,ang_ay,ang_az,ang_vx,ang_vy,ang_vz;
	double uid;

	//Create object
	EVDS_ERRCHECK(EVDS_Object_Create(parent->system,parent,&object));
	if (p_object) *p_object = object;

	//Set parameters
	EVDS_ERRCHECK(SIMC_XML_GetAttribute(doc,root,"name",&name));
	EVDS_ERRCHECK(SIMC_XML_GetAttribute(doc,root,"type",&type));
	EVDS_ERRCHECK(SIMC_XML_GetAttributeDouble(doc,root,"uid",&uid));
	EVDS_ERRCHECK(SIMC_XML_GetAttributeDouble(doc,root,"x",&x));
	EVDS_ERRCHECK(SIMC_XML_GetAttributeDouble(doc,root,"y",&y));
	EVDS_ERRCHECK(SIMC_XML_GetAttributeDouble(doc,root,"z",&z));
	EVDS_ERRCHECK(SIMC_XML_GetAttributeDouble(doc,root,"vx",&vx));
	EVDS_ERRCHECK(SIMC_XML_GetAttributeDouble(doc,root,"vy",&vy));
	EVDS_ERRCHECK(SIMC_XML_GetAttributeDouble(doc,root,"vz",&vz));
	EVDS_ERRCHECK(SIMC_XML_GetAttributeDouble(doc,root,"pitch",&pitch));
	EVDS_ERRCHECK(SIMC_XML_GetAttributeDouble(doc,root,"yaw",&yaw));
	EVDS_ERRCHECK(SIMC_XML_GetAttributeDouble(doc,root,"roll",&roll));

	EVDS_ERRCHECK(SIMC_XML_GetAttributeDouble(doc,root,"q0",&q0));
	EVDS_ERRCHECK(SIMC_XML_GetAttributeDouble(doc,root,"q1",&q1));
	EVDS_ERRCHECK(SIMC_XML_GetAttributeDouble(doc,root,"q2",&q2));
	EVDS_ERRCHECK(SIMC_XML_GetAttributeDouble(doc,root,"q3",&q3));

	EVDS_ERRCHECK(SIMC_XML_GetAttributeDouble(doc,root,"ax",&ax));
	EVDS_ERRCHECK(SIMC_XML_GetAttributeDouble(doc,root,"ay",&ay));
	EVDS_ERRCHECK(SIMC_XML_GetAttributeDouble(doc,root,"az",&az));
	EVDS_ERRCHECK(SIMC_XML_GetAttributeDouble(doc,root,"ang_ax",&ang_ax));
	EVDS_ERRCHECK(SIMC_XML_GetAttributeDouble(doc,root,"ang_ay",&ang_ay));
	EVDS_ERRCHECK(SIMC_XML_GetAttributeDouble(doc,root,"ang_az",&ang_az));
	EVDS_ERRCHECK(SIMC_XML_GetAttributeDouble(doc,root,"ang_vx",&ang_vx));
	EVDS_ERRCHECK(SIMC_XML_GetAttributeDouble(doc,root,"ang_vy",&ang_vy));
	EVDS_ERRCHECK(SIMC_XML_GetAttributeDouble(doc,root,"ang_vz",&ang_vz));

	EVDS_Object_SetPosition(object,parent,x,y,z);
	EVDS_Object_SetVelocity(object,parent,vx,vy,vz);
	EVDS_Object_SetAngularVelocity(object,parent,ang_vx,ang_vy,ang_vz);
	EVDS_Object_SetOrientation(object,parent,EVDS_RAD(roll),EVDS_RAD(pitch),EVDS_RAD(yaw));

	//Special case for specifying attitude as a quaternion
	if ((q0 != 0.0) || (q1 != 0.0) || (q2 != 0.0) || (q3 != 0.0)) {
		EVDS_STATE_VECTOR vector;
		EVDS_Object_GetStateVector(object,&vector);
		vector.orientation.q[0] = q0;
		vector.orientation.q[1] = q1;
		vector.orientation.q[2] = q2;
		vector.orientation.q[3] = q3;
		EVDS_Object_SetStateVector(object,&vector);
	}
	//Special case for accelerations
	if ((ax != 0.0) || (ay != 0.0) || (az != 0.0) ||
		(ang_ax != 0.0) || (ang_ay != 0.0) || (ang_az != 0.0)) {
		EVDS_STATE_VECTOR vector;
		EVDS_Object_GetStateVector(object,&vector);
		EVDS_Vector_Set(&vector.acceleration,EVDS_VECTOR_ACCELERATION,object->parent,ax,ay,az);
		EVDS_Vector_Set(&vector.angular_acceleration,EVDS_VECTOR_ANGULAR_ACCELERATION,object->parent,ang_ax,ang_ay,ang_az);
		EVDS_Object_SetStateVector(object,&vector);
	}

	//Initialize object
	EVDS_Object_SetName(object,name);
	EVDS_Object_SetType(object,type);
	if (uid > 0.0) EVDS_Object_SetUID(object,(unsigned int)uid);

	//Read parameters
	EVDS_ERRCHECK(SIMC_XML_GetElement(doc,root,&element,"parameter"));
	while (element) {
		EVDS_ERRCHECK(EVDS_Internal_LoadParameter(object,0,doc,element,0,info,0));
		EVDS_ERRCHECK(SIMC_XML_Iterate(doc,root,&element,"parameter"));
	}

	//Read all children
	EVDS_ERRCHECK(SIMC_XML_GetElement(doc,root,&element,"object"));
	while (element) {
		EVDS_ERRCHECK(EVDS_Internal_LoadObject(object,doc,element,0,info));
		EVDS_ERRCHECK(SIMC_XML_Iterate(doc,root,&element,"object"));
	}

	//Load all modifiers inside the object
	/*EVDS_ERRCHECK(SIMC_XML_GetElement(doc,root,&element,"modifier"));
	while (element) {
		EVDS_MODIFIER* modifier;
		EVDS_ERRCHECK(EVDS_Internal_LoadModifier(parent,doc,element,&modifier,object));
		//if (info && (!info->skipModifiers)) { FIXME: this information is required
			EVDS_ERRCHECK(EVDS_Modifier_Execute(modifier));
		//}
		EVDS_ERRCHECK(SIMC_XML_Iterate(doc,root,&element,"modifier"));
	}*/
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Load a single "file" (EVDS tag) from XML file
////////////////////////////////////////////////////////////////////////////////
int EVDS_Internal_LoadFile(EVDS_OBJECT* parent, SIMC_XML_DOCUMENT* doc, SIMC_XML_ELEMENT* root, 
						   EVDS_OBJECT** p_object, EVDS_OBJECT_LOADEX* info) {
	SIMC_XML_ELEMENT* element;
	
	//Load all objects inside the file
	if ((info->flags == 0xFFFFFFFF) || (!(info->flags & EVDS_OBJECT_LOADEX_NO_OBJECTS))) {
		EVDS_ERRCHECK(SIMC_XML_GetElement(doc,root,&element,"object"));
		while (element) {
			EVDS_OBJECT* object;
			EVDS_ERRCHECK(EVDS_Internal_LoadObject(parent,doc,element,&object,info));
			if (p_object && (*p_object == 0)) {
				*p_object = object;
			} else if (info->flags != 0xFFFFFFFF) {
				if (!info->firstObject) {
					info->firstObject = object;
					if (info->flags & EVDS_OBJECT_LOADEX_ONLY_FIRST) {
						break;
					}
				}
				if (info->OnLoadObject) {
					EVDS_ERRCHECK(info->OnLoadObject(info,object));
				}
			} else {
				EVDS_Object_Initialize(object,0);
			}

			EVDS_ERRCHECK(SIMC_XML_Iterate(doc,root,&element,"object"));
		}
	}

	//Load all databases
	if ((info->flags == 0xFFFFFFFF) || (!(info->flags & EVDS_OBJECT_LOADEX_NO_DATABASES))) {
		EVDS_ERRCHECK(SIMC_XML_GetElement(doc,root,&element,"database"));
		while (element) {
			char* name;
			SIMC_XML_ELEMENT* nested;
			EVDS_VARIABLE* database;

			//Create new database or find existing one
			EVDS_ERRCHECK(SIMC_XML_GetAttribute(doc,element,"name",&name));
			if (EVDS_System_GetDatabaseByName(parent->system,name,&database) != EVDS_OK) {
				EVDS_ERRCHECK(EVDS_Variable_Create(parent->system,name,EVDS_VARIABLE_TYPE_NESTED,&database));
				SIMC_List_Append(parent->system->databases,database);
			}

			//Load all entries
			EVDS_ERRCHECK(SIMC_XML_GetElement(doc,element,&nested,"entry"));
			while (nested) {
				EVDS_ERRCHECK(EVDS_Internal_LoadParameter(0,database,doc,nested,0,info,0));
				EVDS_ERRCHECK(SIMC_XML_Iterate(doc,element,&nested,"entry"));
			}

			EVDS_ERRCHECK(SIMC_XML_Iterate(doc,root,&element,"database"));
		}
	}

	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Parse all objects/files from an XML document
////////////////////////////////////////////////////////////////////////////////
int EVDS_Internal_ParseFile(EVDS_OBJECT* parent, SIMC_XML_DOCUMENT* doc, 
							EVDS_OBJECT** p_object, EVDS_OBJECT_LOADEX* info) {
	EVDS_OBJECT_LOADEX default_info = { 0 };
	SIMC_XML_ELEMENT* root;
	SIMC_XML_ELEMENT* element;
	int version;

	//Use default information structure
	if (!info) info = &default_info;
	default_info.flags = 0xFFFFFFFF;

	//Read file version
	EVDS_ERRCHECK(SIMC_XML_GetRootElement(doc,&root,"EVDS"));
	EVDS_ERRCHECK(SIMC_XML_GetAttributeInt(doc,root,"version",&version));
	info->version = version;

	//Load single object or multiple objects
	if (root) {
		//Load a single object
		EVDS_ERRCHECK(EVDS_Internal_LoadFile(parent,doc,root,p_object,info));
	} else {
		//Load all objects inside data file
		EVDS_ERRCHECK(SIMC_XML_GetRootElement(doc,&root,"DATA"));
		EVDS_ERRCHECK(SIMC_XML_GetElement(doc,root,&element,"EVDS"));
		EVDS_ERRCHECK(SIMC_XML_GetAttributeInt(doc,root,"version",&version));
		info->version = version; //Get version anyway
		while (element) {
			EVDS_ERRCHECK(EVDS_Internal_LoadFile(parent,doc,element,p_object,info));
			EVDS_ERRCHECK(SIMC_XML_Iterate(doc,root,&element,"EVDS"));
		}
	}

	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Load object from XML file and return the first object in file.
///
/// The first returned object will not be initialized. All following objects (if file
/// contains more than one object) will be automatically initialized in a non-blocking
/// way.
///
/// This function is usually used to load a single object, for example:
///
///		EVDS_Object_LoadFromFile(inertial_system,"sat_test.evds",&satellite);
///		EVDS_Object_Initialize(satellite,1);
///
/// If no pointer is passed into the function, the first object (and any following it) will
/// all be automatically initialized. A valid parent object must be passed into the function,
/// it cannot be a null pointer.
///
/// Errors from automatic initialization will not be reported by this function. Use
/// EVDS_LoadEx() to properly load files with several objects defined in them.
///
/// @param[in] parent Parent object
/// @param[in] filename Pointer to a null-terminated filename
/// @param[out] p_object Pointer to first object will be written here (can be null)
///
/// @returns Error code
/// @retval EVDS_OK Successfully completed
///	@retval EVDS_ERROR_BAD_PARAMETER "filename" is null
/// @retval EVDS_ERROR_FILE File could not be opened (not found or not accessible)
/// @retval EVDS_ERROR_SYNTAX Syntax error in input file
/// @retval EVDS_ERROR_MEMORY Could not allocate memory for EVDS_OBJECT
/// @retval EVDS_ERROR_MEMORY Could not allocate memory for EVDS_VARIABLE
////////////////////////////////////////////////////////////////////////////////
int EVDS_Object_LoadFromFile(EVDS_OBJECT* parent, const char* filename, EVDS_OBJECT** p_object) {
	SIMC_XML_DOCUMENT* doc;
	if (!parent) return EVDS_ERROR_BAD_PARAMETER;
	if (!filename) return EVDS_ERROR_BAD_PARAMETER;
	if (p_object) *p_object = 0;

	EVDS_ERRCHECK(SIMC_XML_Open(filename,&doc,0,0));
	EVDS_ERRCHECK(EVDS_Internal_ParseFile(parent,doc,p_object,0));
	EVDS_ERRCHECK(SIMC_XML_Close(doc));
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Empty callbacks for EVDS_Object_LoadEx()
////////////////////////////////////////////////////////////////////////////////
int EVDS_Internal_OnLoadObject(EVDS_OBJECT_LOADEX* info, EVDS_OBJECT* object) {
	return EVDS_OK;
}
int EVDS_Internal_OnSyntaxError(EVDS_OBJECT_LOADEX* info, const char* error) {
	return EVDS_OK;
}
EVDS_OBJECT_LOADEX EVDS_Internal_LoadEx = {
	EVDS_Internal_OnLoadObject,
	EVDS_Internal_OnSyntaxError,
};


////////////////////////////////////////////////////////////////////////////////
/// @brief Load objects from file or description string in XML format.
///
/// This function is used to load multiple objects from a file. All objects must
/// be manually initialized by the user inside the callback.
/// If null pointer is passed for info structure, no objects will be automatically
/// initialized. A valid parent object must be passed into the function.
///
/// @c OnLoadObject callback will be called for every object stored directly in the loaded
/// description (it is only called for objects under EVDS XML tag - but not for these objects
/// children).
///
/// @c OnSyntaxError callback is called if the description contains XML syntax errors.
///
/// This function is used to load multiple objects, for example:
/// ~~~{.c}
///		int EVDS_Internal_OnLoadObject(EVDS_SYSTEM* system, EVDS_OBJECT_LOADEX* info, EVDS_OBJECT* object) {
///			EVDS_Object_Initialize(object,1);
///			return EVDS_OK;
///		}
///		int EVDS_Internal_OnSyntaxError(EVDS_SYSTEM* system, EVDS_OBJECT_LOADEX* info, const char* error) {
///			printf("Syntax error in %s: %s\n",info->userdata,error);
///			return EVDS_OK;
///		}
///		EVDS_OBJECT_LOADEX info = { OnLoadFile, OnSyntaxError, 0, "sat_test.evds" };
///		EVDS_Object_LoadEx(inertial_system,"sat_test.evds",&info);
///	~~~
///
/// @param[in] parent Parent object
/// @param[in] filename Pointer to a null-terminated filename (or null if loading from description)
/// @param[in] info Pointer to EVDS_OBJECT_LOADEX structure (can be null)
///
/// @returns Error code or error code from custom callback
/// @retval EVDS_OK Successfully completed
///	@retval EVDS_ERROR_BAD_PARAMETER "parent" is null
///	@retval EVDS_ERROR_BAD_PARAMETER "filename" and "info->description" are both null
/// @retval EVDS_ERROR_FILE File could not be opened (not found or not accessible)
/// @retval EVDS_ERROR_SYNTAX Syntax error in input file or description
/// @retval EVDS_ERROR_MEMORY Could not allocate memory for EVDS_OBJECT
/// @retval EVDS_ERROR_MEMORY Could not allocate memory for EVDS_VARIABLE
////////////////////////////////////////////////////////////////////////////////
int EVDS_Object_LoadEx(EVDS_OBJECT* parent, const char* filename, EVDS_OBJECT_LOADEX* info) {
	SIMC_XML_DOCUMENT* doc;
	if (!parent) return EVDS_ERROR_BAD_PARAMETER;
	if (!info) info = &EVDS_Internal_LoadEx;
	if ((!info->description) && (!filename)) return EVDS_ERROR_BAD_PARAMETER;

	if (info->description) {
		EVDS_ERRCHECK(SIMC_XML_OpenString(info->description,&doc,info->OnSyntaxError,info));
	} else {
		EVDS_ERRCHECK(SIMC_XML_Open(filename,&doc,info->OnSyntaxError,info));		
	}
	EVDS_ERRCHECK(EVDS_Internal_ParseFile(parent,doc,0,info));
	EVDS_ERRCHECK(SIMC_XML_Close(doc));
	return EVDS_OK;
}


///////////////////////////////////////////////////////////////////////////////
/// @brief Load objects from an XML string and return the first object loaded.
///
/// The first returned object will not be initialized. All following objects (if file
/// contains more than one object) will be automatically initialized in a non-blocking
/// way.
///
/// This function is usually used to load a single object, for example:
///
///		EVDS_Object_LoadFromString(inertial_system,
///			"<EVDS>"
///			"  <object type=\"vessel\" name=\"Satellite\">"
///			"    <parameter name=\"mass\">1000</parameter>"
///			"    <parameter name=\"ixx\">100</parameter>"
///			"    <parameter name=\"iyy\">1000</parameter>"
///			"    <parameter name=\"izz\">500</parameter>"
///			"    <parameter name=\"cm\">1.0 0.0 -0.5</parameter>"
///			"  </object>"
///			"</EVDS>",&satellite);
///		EVDS_Object_Initialize(satellite,1);
///
/// If no pointer is passed into the function, the first object (and any following it) will
/// all be automatically initialized. A valid parent object must be passed into the function,
/// it cannot be a null pointer.
///
/// Errors from automatic initialization will not be reported by this function. Use
/// EVDS_LoadEx() to properly load files with several objects defined in them.
///
/// @param[in] parent Parent object
/// @param[in] description Pointer to a null-terminated model description
/// @param[out] p_object Pointer to first object will be written here (can be null)
///
/// @returns Error code
/// @retval EVDS_OK Successfully completed
///	@retval EVDS_ERROR_BAD_PARAMETER "parent" is null
///	@retval EVDS_ERROR_BAD_PARAMETER "description" is null
/// @retval EVDS_ERROR_SYNTAX Syntax error in description
/// @retval EVDS_ERROR_MEMORY Could not allocate memory for EVDS_OBJECT
/// @retval EVDS_ERROR_MEMORY Could not allocate memory for EVDS_VARIABLE
///////////////////////////////////////////////////////////////////////////////
int EVDS_Object_LoadFromString(EVDS_OBJECT* parent, const char* description, EVDS_OBJECT** p_object) {
	SIMC_XML_DOCUMENT* doc;

	if (p_object) *p_object = 0;
	EVDS_ERRCHECK(SIMC_XML_OpenString(description,&doc,0,0));
	EVDS_ERRCHECK(EVDS_Internal_ParseFile(parent,doc,p_object,0));
	EVDS_ERRCHECK(SIMC_XML_Close(doc));
	return EVDS_OK;
}
