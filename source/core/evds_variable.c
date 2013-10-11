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
#include "evds.h"


////////////////////////////////////////////////////////////////////////////////
/// @brief Create new variable
////////////////////////////////////////////////////////////////////////////////
int EVDS_Variable_Create(EVDS_SYSTEM* system, const char* name, EVDS_VARIABLE_TYPE type, EVDS_VARIABLE** p_variable) {
	EVDS_VARIABLE* variable;
	if (!system) return EVDS_ERROR_BAD_PARAMETER;
	if (!name) return EVDS_ERROR_BAD_PARAMETER;
	if (!p_variable) return EVDS_ERROR_BAD_PARAMETER;

	//Create variable
	variable = (EVDS_VARIABLE*)malloc(sizeof(EVDS_VARIABLE));
	*p_variable = variable;
	if (!variable) return EVDS_ERROR_MEMORY;
	memset(variable,0,sizeof(EVDS_VARIABLE));
	
	//Setup the variable
	variable->system = system;
	variable->type = type;
	EVDS_Variable_SetName(variable,name);

	//Initialize to the given type
	switch (type) {
		case EVDS_VARIABLE_TYPE_FLOAT:
			variable->value_size = sizeof(EVDS_REAL);
			variable->value = (EVDS_REAL*)malloc(sizeof(EVDS_REAL));
		break;
		case EVDS_VARIABLE_TYPE_STRING:
			variable->value_size = 1;
			variable->value = (char*)malloc(sizeof(char));
#ifndef EVDS_SINGLETHREADED
			variable->lock = SIMC_Lock_Create();
#endif
		break;
		case EVDS_VARIABLE_TYPE_VECTOR:
			variable->value_size = sizeof(EVDS_VECTOR);
			variable->value = (EVDS_VECTOR*)malloc(sizeof(EVDS_VECTOR));
		break;
		case EVDS_VARIABLE_TYPE_QUATERNION:
			variable->value_size = sizeof(EVDS_QUATERNION);
			variable->value = (EVDS_QUATERNION*)malloc(sizeof(EVDS_QUATERNION));
		break;
		case EVDS_VARIABLE_TYPE_NESTED:
			variable->value_size = 1;
			variable->value = (char*)malloc(sizeof(char));
#ifndef EVDS_SINGLETHREADED
			variable->lock = SIMC_Lock_Create();
#endif
			SIMC_List_Create(&variable->attributes,0);
			SIMC_List_Create(&variable->list,0);
		break;
		case EVDS_VARIABLE_TYPE_DATA_PTR:
		case EVDS_VARIABLE_TYPE_FUNCTION_PTR:
			variable->value_size = 0;
			variable->value = 0;
		break;
		case EVDS_VARIABLE_TYPE_FUNCTION:
			variable->value_size = sizeof(EVDS_VARIABLE_FUNCTION);
			variable->value = (EVDS_VARIABLE_FUNCTION*)malloc(sizeof(EVDS_VARIABLE_FUNCTION));
			SIMC_List_Create(&variable->list,0);
		break;
	}

	//Clear out variable value
	if (variable->value) {
		memset(variable->value,0,variable->value_size);
	}
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Create a new variable as a copy of existing one
////////////////////////////////////////////////////////////////////////////////
int EVDS_Variable_Copy(EVDS_VARIABLE* source, EVDS_VARIABLE* variable) {
	//int error_code;
	if (!source) return EVDS_ERROR_BAD_PARAMETER;
	if (!variable) return EVDS_ERROR_BAD_PARAMETER;

	//Create
	strncpy(variable->name,source->name,64);
	variable->type = source->type;

	//Copy value
	switch (variable->type) {
		case EVDS_VARIABLE_TYPE_FLOAT: {
			EVDS_REAL value;
			EVDS_Variable_GetReal(source,&value);
			EVDS_Variable_SetReal(variable,value);
		} break;
		case EVDS_VARIABLE_TYPE_STRING: {
			size_t length;
			char* value;
			EVDS_Variable_GetString(source,0,0,&length);
			value = (char*)alloca(length);
			EVDS_Variable_GetString(source,value,length,0);
			EVDS_Variable_SetString(variable,value,length);
		} break;
		case EVDS_VARIABLE_TYPE_VECTOR: {
			EVDS_VECTOR value;
			EVDS_Variable_GetVector(source,&value);
			EVDS_Variable_SetVector(variable,&value);
		} break;
		case EVDS_VARIABLE_TYPE_QUATERNION: {
			EVDS_QUATERNION value;
			EVDS_Variable_GetQuaternion(source,&value);
			EVDS_Variable_SetQuaternion(variable,&value);
		} break;
		case EVDS_VARIABLE_TYPE_NESTED: {
			size_t length;
			char* string_value;
			char name[65];
			SIMC_LIST_ENTRY* entry;
			EVDS_VARIABLE* source_value;
			EVDS_VARIABLE* value;

			entry = SIMC_List_GetFirst(source->attributes);
			while (entry) {
				source_value = (EVDS_VARIABLE*)SIMC_List_GetData(source->attributes,entry);
				strncpy(name,source_value->name,64); name[64] = 0;

				EVDS_Variable_AddAttribute(variable,name,source_value->type,&value);
				EVDS_Variable_Copy(source_value,value);

				entry = SIMC_List_GetNext(source->attributes,entry);
			}

			entry = SIMC_List_GetFirst(source->list);
			while (entry) {
				source_value = (EVDS_VARIABLE*)SIMC_List_GetData(source->list,entry);
				strncpy(name,source_value->name,64); name[64] = 0;
				
				EVDS_Variable_AddNested(variable,name,source_value->type,&value);
				EVDS_Variable_Copy(source_value,value);

				entry = SIMC_List_GetNext(source->list,entry);
			}

			//Copy embedded string
			EVDS_Variable_GetString(source,0,0,&length);
			string_value = (char*)alloca(length);
			EVDS_Variable_GetString(source,string_value,length,0);
			EVDS_Variable_SetString(variable,string_value,length);
		} break;
		case EVDS_VARIABLE_TYPE_DATA_PTR:
		case EVDS_VARIABLE_TYPE_FUNCTION_PTR: {
			variable->value = source->value;
		} break;
		case EVDS_VARIABLE_TYPE_FUNCTION: {
			//FIXME
		} break;
	}

	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Destroys data of a given variable (used internally)
////////////////////////////////////////////////////////////////////////////////
int EVDS_InternalVariable_DestroyData(EVDS_VARIABLE* variable) {
	SIMC_LIST_ENTRY* entry;
	if (!variable) return EVDS_ERROR_BAD_PARAMETER;

	//Delete variable from lists (if has parent)
	if (variable->parent) {
		if (variable->attribute_entry) {
			SIMC_List_GetFirst(variable->parent->attributes);
			SIMC_List_Remove(variable->parent->attributes,variable->attribute_entry);
		}
		if (variable->list_entry) {
			SIMC_List_GetFirst(variable->parent->list);
			SIMC_List_Remove(variable->parent->list,variable->list_entry);
		}
	} else { //Delete from object list
		if (variable->list_entry) {
			SIMC_List_GetFirst(variable->object->variables);
			SIMC_List_Remove(variable->object->variables,variable->list_entry);
		}
	}
		
	//Delete all attributes
	if (variable->attributes) {
		entry = SIMC_List_GetFirst(variable->attributes);
		while (entry) {
			EVDS_InternalVariable_DestroyData((EVDS_VARIABLE*)SIMC_List_GetData(variable->attributes,entry));
			entry = SIMC_List_GetFirst(variable->attributes);
		}
		SIMC_List_Destroy(variable->attributes);
	}

	//Delete all nested variables
	if (variable->list) {
		entry = SIMC_List_GetFirst(variable->list);
		while (entry) {
			EVDS_InternalVariable_DestroyData((EVDS_VARIABLE*)SIMC_List_GetData(variable->list,entry));
			entry = SIMC_List_GetFirst(variable->list);
		}
		SIMC_List_Destroy(variable->list);
	}

	//Delete resources according to variable type
	if (variable->value) {
		switch (variable->type) {
			case EVDS_VARIABLE_TYPE_FLOAT:
			case EVDS_VARIABLE_TYPE_STRING:
			case EVDS_VARIABLE_TYPE_VECTOR:
			case EVDS_VARIABLE_TYPE_QUATERNION:
				free(variable->value);
			break;
			case EVDS_VARIABLE_TYPE_DATA_PTR:
			case EVDS_VARIABLE_TYPE_FUNCTION_PTR:
			break;
			case EVDS_VARIABLE_TYPE_FUNCTION:
				EVDS_InternalVariable_DestroyFunction(variable,variable->value);
				free(variable->value);
			break;
		}
	}
#ifndef EVDS_SINGLETHREADED
	if (variable->lock) SIMC_Lock_Destroy(variable->lock);
#endif
	free(variable);
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Destroy a variable or attribute. @evds_init_only
///
/// @param[in] variable Variable, which must be destroyed
///
/// @returns Error code
/// @retval EVDS_OK Successfully completed
/// @retval EVDS_ERROR_BAD_PARAMETER "variable" is null
/// @retval EVDS_ERROR_BAD_STATE Object was already initialized
/// @retval EVDS_ERROR_INVALID_OBJECT Object was destroyed
/// @retval EVDS_ERROR_INTERTHREAD_CALL The function can only be called from thread that is initializing the object 
///  (or thread that has created the object before initializer was called)
////////////////////////////////////////////////////////////////////////////////
int EVDS_Variable_Destroy(EVDS_VARIABLE* variable) {
	if (!variable) return EVDS_ERROR_BAD_PARAMETER;
#ifndef EVDS_SINGLETHREADED
	if (variable->object) {
		if (variable->object->destroyed) return EVDS_ERROR_INVALID_OBJECT;
		if (variable->object->initialized) return EVDS_ERROR_BAD_STATE;
		if ((variable->object->create_thread != SIMC_Thread_GetUniqueID()) &&
			(variable->object->initialize_thread != SIMC_Thread_GetUniqueID())) return EVDS_ERROR_INTERTHREAD_CALL;
	}
#endif

	return EVDS_InternalVariable_DestroyData(variable);
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Move variable in parent variables list. @evds_init_only
///
/// @param[in] variable Pointer to variable which must be moved
/// @param[in] head Pointer to variable which will be in front in list (can be null)
///
/// @returns Error code
/// @retval EVDS_OK Successfully completed
/// @retval EVDS_ERROR_BAD_PARAMETER "variable" is null
/// @retval EVDS_ERROR_BAD_PARAMETER "head" does not have same parent as "variable"
/// @retval EVDS_ERROR_BAD_PARAMETER "variable" is not in parents list of nested variables
/// @retval EVDS_ERROR_BAD_PARAMETER "head" is not in parents list of nested variables
/// @retval EVDS_ERROR_BAD_PARAMETER "variable" has no parent
/// @retval EVDS_ERROR_BAD_STATE Object was already initialized
/// @retval EVDS_ERROR_INVALID_OBJECT Object was destroyed
/// @retval EVDS_ERROR_INTERTHREAD_CALL The function can only be called from thread that is initializing the object 
///  (or thread that has created the object before initializer was called)
////////////////////////////////////////////////////////////////////////////////
int EVDS_Variable_MoveInList(EVDS_VARIABLE* variable, EVDS_VARIABLE* head) {
	SIMC_LIST_ENTRY* head_entry = 0;
	if (!variable) return EVDS_ERROR_BAD_PARAMETER;
	if (!variable->parent) return EVDS_ERROR_BAD_PARAMETER;
	if (head && (head->parent != variable->parent)) return EVDS_ERROR_BAD_PARAMETER;
	if (!variable->list_entry) return EVDS_ERROR_BAD_PARAMETER;
	if (head && (!head->list_entry)) return EVDS_ERROR_BAD_PARAMETER;
#ifndef EVDS_SINGLETHREADED
	if (variable->object) {
		if (variable->object->destroyed) return EVDS_ERROR_INVALID_OBJECT;
		if (variable->object->initialized) return EVDS_ERROR_BAD_STATE;
		if ((variable->object->create_thread != SIMC_Thread_GetUniqueID()) &&
			(variable->object->initialize_thread != SIMC_Thread_GetUniqueID())) return EVDS_ERROR_INTERTHREAD_CALL;
	}
#endif

	if (head) head_entry = head->list_entry;
	SIMC_List_GetFirst(variable->parent->list);
	SIMC_List_MoveInFront(variable->parent->list,variable->list_entry,head_entry);
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Add a nested variable. @evds_init_only
///
/// Arbitrary typed variables (type is EVDS_VARIABLE_TYPE_NESTED) may have another
/// variables inside them. This can be used to represent table and matrix data.
///
/// Functions are not nested, but can contain nested entries which represent data
/// arrays or configuration/information for interpolation routines.
///
/// @param[in] parent_variable Variable, to which a new one must be added
/// @param[in] name Name of a new nested variable
/// @param[in] type Type of a new nested variable
/// @param[out] p_variable Pointer to new variable will be written here
///
/// @returns Error code
/// @retval EVDS_OK Successfully completed
/// @retval EVDS_ERROR_BAD_PARAMETER "name" is null
/// @retval EVDS_ERROR_BAD_PARAMETER "p_variable" is null
/// @retval EVDS_ERROR_BAD_PARAMETER "parent_variable" is null
/// @retval EVDS_ERROR_BAD_STATE Parent variable type is not EVDS_VARIABLE_TYPE_NESTED
/// @retval EVDS_ERROR_BAD_STATE Object was already initialized
/// @retval EVDS_ERROR_INVALID_OBJECT Object was destroyed
/// @retval EVDS_ERROR_INTERTHREAD_CALL The function can only be called from thread that is initializing the object 
///  (or thread that has created the object before initializer was called)
/// @retval EVDS_ERROR_MEMORY Error allocating EVDS_VARIABLE data structure
////////////////////////////////////////////////////////////////////////////////
int EVDS_Variable_AddNested(EVDS_VARIABLE* parent_variable, const char* name, EVDS_VARIABLE_TYPE type, EVDS_VARIABLE** p_variable) {
	int error_code;
	if (!name) return EVDS_ERROR_BAD_PARAMETER;
	if (!p_variable) return EVDS_ERROR_BAD_PARAMETER;
	if (!parent_variable) return EVDS_ERROR_BAD_PARAMETER;
	if ((parent_variable->type != EVDS_VARIABLE_TYPE_NESTED) &&
		(parent_variable->type != EVDS_VARIABLE_TYPE_FUNCTION)) return EVDS_ERROR_BAD_STATE;
#ifndef EVDS_SINGLETHREADED
	if (parent_variable->object) {
		if (parent_variable->object->destroyed) return EVDS_ERROR_INVALID_OBJECT;
		if (parent_variable->object->initialized) return EVDS_ERROR_BAD_STATE;
		if ((parent_variable->object->create_thread != SIMC_Thread_GetUniqueID()) &&
			(parent_variable->object->initialize_thread != SIMC_Thread_GetUniqueID())) return EVDS_ERROR_INTERTHREAD_CALL;
	}
#endif

	//Create variable
	error_code = EVDS_Variable_Create(parent_variable->system,name,type,p_variable);
	if (error_code == EVDS_OK) {
		(*p_variable)->parent = parent_variable;
		(*p_variable)->object = parent_variable->object;
		(*p_variable)->list_entry = SIMC_List_Append(parent_variable->list,(*p_variable));
	}
	return error_code;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Add an attribute. @evds_init_only
///
/// Attributes are additional modifiers for the arbitrary data type. The are parameters
/// of the parent variable itself
///
/// @param[in] parent_variable Variable, to which an attribute must be added
/// @param[in] name Name of an attribute
/// @param[in] type Type of an attribute
/// @param[out] p_variable Pointer to an attribute will be written here
///
/// @returns Error code
/// @retval EVDS_OK Successfully completed
/// @retval EVDS_ERROR_BAD_PARAMETER "name" is null
/// @retval EVDS_ERROR_BAD_PARAMETER "p_variable" is null
/// @retval EVDS_ERROR_BAD_PARAMETER "parent_variable" is null
/// @retval EVDS_ERROR_BAD_STATE Parent variable type is not EVDS_VARIABLE_TYPE_NESTED
/// @retval EVDS_ERROR_BAD_STATE Object was already initialized
/// @retval EVDS_ERROR_INVALID_OBJECT Object was destroyed
/// @retval EVDS_ERROR_INTERTHREAD_CALL The function can only be called from thread that is initializing the object 
///  (or thread that has created the object before initializer was called)
/// @retval EVDS_ERROR_MEMORY Error allocating EVDS_VARIABLE data structure
////////////////////////////////////////////////////////////////////////////////
int EVDS_Variable_AddAttribute(EVDS_VARIABLE* parent_variable, const char* name, EVDS_VARIABLE_TYPE type, EVDS_VARIABLE** p_variable) {
	int error_code;
	if (!name) return EVDS_ERROR_BAD_PARAMETER;
	if (!p_variable) return EVDS_ERROR_BAD_PARAMETER;
	if (!parent_variable) return EVDS_ERROR_BAD_PARAMETER;
	if (parent_variable->type != EVDS_VARIABLE_TYPE_NESTED) return EVDS_ERROR_BAD_STATE;
#ifndef EVDS_SINGLETHREADED
	if (parent_variable->object) {
		if (parent_variable->object->destroyed) return EVDS_ERROR_INVALID_OBJECT;
		if (parent_variable->object->initialized) return EVDS_ERROR_BAD_STATE;
		if ((parent_variable->object->create_thread != SIMC_Thread_GetUniqueID()) &&
			(parent_variable->object->initialize_thread != SIMC_Thread_GetUniqueID())) return EVDS_ERROR_INTERTHREAD_CALL;
	}
#endif

	error_code = EVDS_Variable_GetAttribute(parent_variable,name,p_variable);
	if (error_code == EVDS_ERROR_NOT_FOUND) { //Create variable
		error_code = EVDS_Variable_Create(parent_variable->system,name,type,p_variable);
		if (error_code == EVDS_OK) {
			(*p_variable)->parent = parent_variable;
			(*p_variable)->object = parent_variable->object;
			(*p_variable)->attribute_entry = SIMC_List_Append(parent_variable->attributes,(*p_variable));
		}
	}
	return error_code;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Add a floating-point attribute. @evds_init_only
///
/// Attributes are additional modifiers for the arbitrary data type. The are parameters
/// of the parent variable itself
///
/// @param[in] parent_variable Variable, to which an attribute must be added
/// @param[in] name Name of an attribute
/// @param[in] value Value of the floating-point attribute
/// @param[out] p_variable Pointer to an attribute will be written here (can be null)
///
/// @returns Error code
/// @retval EVDS_OK Successfully completed
/// @retval EVDS_ERROR_BAD_PARAMETER "name" is null
/// @retval EVDS_ERROR_BAD_PARAMETER "parent_variable" is null
/// @retval EVDS_ERROR_BAD_STATE Parent variable type is not EVDS_VARIABLE_TYPE_NESTED
/// @retval EVDS_ERROR_BAD_STATE Object was already initialized
/// @retval EVDS_ERROR_INVALID_OBJECT Object was destroyed
/// @retval EVDS_ERROR_INTERTHREAD_CALL The function can only be called from thread that is initializing the object 
///  (or thread that has created the object before initializer was called)
/// @retval EVDS_ERROR_MEMORY Error allocating EVDS_VARIABLE data structure
////////////////////////////////////////////////////////////////////////////////
int EVDS_Variable_AddFloatAttribute(EVDS_VARIABLE* parent_variable, const char* name, EVDS_REAL value, EVDS_VARIABLE** p_variable) {
	int error_code;
	EVDS_VARIABLE* variable;
	if (!name) return EVDS_ERROR_BAD_PARAMETER;
	if (!parent_variable) return EVDS_ERROR_BAD_PARAMETER;
	if (parent_variable->type != EVDS_VARIABLE_TYPE_NESTED) return EVDS_ERROR_BAD_STATE;
#ifndef EVDS_SINGLETHREADED
	if (parent_variable->object) {
		if (parent_variable->object->destroyed) return EVDS_ERROR_INVALID_OBJECT;
		if (parent_variable->object->initialized) return EVDS_ERROR_BAD_STATE;
		if ((parent_variable->object->create_thread != SIMC_Thread_GetUniqueID()) &&
			(parent_variable->object->initialize_thread != SIMC_Thread_GetUniqueID())) return EVDS_ERROR_INTERTHREAD_CALL;
	}
#endif

	//Get variable
	error_code = EVDS_Variable_GetAttribute(parent_variable,name,&variable);
	if (error_code == EVDS_ERROR_NOT_FOUND) {
		error_code = EVDS_Variable_AddAttribute(parent_variable,name,EVDS_VARIABLE_TYPE_FLOAT,&variable);
		if (error_code != EVDS_OK) return error_code;
		error_code = EVDS_Variable_SetReal(variable,value);
		if (error_code != EVDS_OK) return error_code;
	}

	if (p_variable) *p_variable = variable;
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Get an attribute from a variable. @evds_limited_init
///
/// See EVDS_Variable_AddAttribute() for more information.
///
/// @param[in] parent_variable Variable, from which attribute must be retrived
/// @param[in] name Name of an attribute
/// @param[out] p_variable Pointer to an attribute will be written here
///
/// @returns Error code
/// @retval EVDS_OK Successfully completed
/// @retval EVDS_ERROR_BAD_PARAMETER "name" is null
/// @retval EVDS_ERROR_BAD_PARAMETER "p_variable" is null
/// @retval EVDS_ERROR_BAD_PARAMETER "parent_variable" is null
/// @retval EVDS_ERROR_BAD_STATE Parent variable type is not EVDS_VARIABLE_TYPE_NESTED
/// @retval EVDS_ERROR_BAD_STATE Object was already initialized
/// @retval EVDS_ERROR_INVALID_OBJECT Object was destroyed
/// @retval EVDS_ERROR_INTERTHREAD_CALL The function can only be called from thread that is initializing the object 
///  (or thread that has created the object before initializer was called)
////////////////////////////////////////////////////////////////////////////////
int EVDS_Variable_GetAttribute(EVDS_VARIABLE* parent_variable, const char* name, EVDS_VARIABLE** p_variable) {
	SIMC_LIST_ENTRY* entry;
	if (!name) return EVDS_ERROR_BAD_PARAMETER;
	if (!p_variable) return EVDS_ERROR_BAD_PARAMETER;
	if (!parent_variable) return EVDS_ERROR_BAD_PARAMETER;
	if (parent_variable->type != EVDS_VARIABLE_TYPE_NESTED) return EVDS_ERROR_BAD_STATE;
#ifndef EVDS_SINGLETHREADED
	if (parent_variable->object) {
		if (parent_variable->object->destroyed) return EVDS_ERROR_INVALID_OBJECT;
		if (!parent_variable->object->initialized &&
			(parent_variable->object->create_thread != SIMC_Thread_GetUniqueID()) &&
			(parent_variable->object->initialize_thread != SIMC_Thread_GetUniqueID())) return EVDS_ERROR_INTERTHREAD_CALL;
	}
#endif

	entry = SIMC_List_GetFirst(parent_variable->attributes);
	while (entry) {
		EVDS_VARIABLE* variable = SIMC_List_GetData(parent_variable->attributes,entry);
		if (strncmp(name,variable->name,64) == 0) {
			*p_variable = variable;
			SIMC_List_Stop(parent_variable->attributes,entry);
			return EVDS_OK;
		}
		entry = SIMC_List_GetNext(parent_variable->attributes,entry);
	}
	return EVDS_ERROR_NOT_FOUND;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Get a nested variable from a variable. @evds_limited_init
///
/// See EVDS_Variable_AddNested() for more information.
///
/// @param[in] parent_variable Variable, from which nested variable must be retrived
/// @param[in] name Name of an attribute
/// @param[out] p_variable Pointer to a nested variable will be written here
///
/// @returns Error code
/// @retval EVDS_OK Successfully completed
/// @retval EVDS_ERROR_BAD_PARAMETER "name" is null
/// @retval EVDS_ERROR_BAD_PARAMETER "p_variable" is null
/// @retval EVDS_ERROR_BAD_PARAMETER "parent_variable" is null
/// @retval EVDS_ERROR_BAD_STATE Parent variable type is not EVDS_VARIABLE_TYPE_NESTED
/// @retval EVDS_ERROR_BAD_STATE Object was already initialized
/// @retval EVDS_ERROR_INVALID_OBJECT Object was destroyed
/// @retval EVDS_ERROR_INTERTHREAD_CALL The function can only be called from thread that is initializing the object 
///  (or thread that has created the object before initializer was called)
////////////////////////////////////////////////////////////////////////////////
int EVDS_Variable_GetNested(EVDS_VARIABLE* parent_variable, const char* name, EVDS_VARIABLE** p_variable) {
	SIMC_LIST_ENTRY* entry;
	if (!name) return EVDS_ERROR_BAD_PARAMETER;
	if (!p_variable) return EVDS_ERROR_BAD_PARAMETER;
	if (!parent_variable) return EVDS_ERROR_BAD_PARAMETER;
	if ((parent_variable->type != EVDS_VARIABLE_TYPE_NESTED) &&
		(parent_variable->type != EVDS_VARIABLE_TYPE_FUNCTION)) return EVDS_ERROR_BAD_STATE;
#ifndef EVDS_SINGLETHREADED
	if (parent_variable->object) {
		if (parent_variable->object->destroyed) return EVDS_ERROR_INVALID_OBJECT;
		if (!parent_variable->object->initialized &&
			(parent_variable->object->create_thread != SIMC_Thread_GetUniqueID()) &&
			(parent_variable->object->initialize_thread != SIMC_Thread_GetUniqueID())) return EVDS_ERROR_INTERTHREAD_CALL;
	}
#endif

	entry = SIMC_List_GetFirst(parent_variable->list);
	while (entry) {
		EVDS_VARIABLE* variable = SIMC_List_GetData(parent_variable->list,entry);
		if (strncmp(name,variable->name,64) == 0) {
			*p_variable = variable;
			SIMC_List_Stop(parent_variable->list,entry);
			return EVDS_OK;
		}
		entry = SIMC_List_GetNext(parent_variable->list,entry);
	}
	return EVDS_ERROR_NOT_FOUND;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Get a variables name. @evds_limited_init
///
/// Returns a string no more than max_length characters long. It may not be null
/// terminated. This is a correct way to get full name:
///
///		char name[257]; //256 plus null terminator
///		EVDS_Variable_GetName(variable,name,256);
///		name[256] = '\0'; //Null-terminate name
///
/// @param[in] variable Variable, name of which will be returned
/// @param[out] name Pointer to name string
/// @param[in] max_length Maximum length of name string (no more than 256 needed)
///
/// @returns Error code
/// @retval EVDS_OK Successfully completed
/// @retval EVDS_ERROR_BAD_PARAMETER "variable" is null
/// @retval EVDS_ERROR_BAD_PARAMETER "name" is null
/// @retval EVDS_ERROR_INVALID_OBJECT Object was destroyed
/// @retval EVDS_ERROR_INTERTHREAD_CALL The function can only be called from thread that is initializing the object 
///  (or thread that has created the object before initializer was called)
////////////////////////////////////////////////////////////////////////////////
int EVDS_Variable_GetName(EVDS_VARIABLE* variable, char* name, size_t max_length) {
	if (!variable) return EVDS_ERROR_BAD_PARAMETER;
	if (!name) return EVDS_ERROR_BAD_PARAMETER;
#ifndef EVDS_SINGLETHREADED
	if (variable->object) {
		if (variable->object->destroyed) return EVDS_ERROR_INVALID_OBJECT;
		if (!variable->object->initialized &&
			(variable->object->create_thread != SIMC_Thread_GetUniqueID()) &&
			(variable->object->initialize_thread != SIMC_Thread_GetUniqueID())) return EVDS_ERROR_INTERTHREAD_CALL;
	}
#endif

	strncpy(name,variable->name,(max_length > 256 ? 256 : max_length));
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Sets variables name. @evds_init_only
///
/// Name must be a null-terminated C string, or a string of 256 characters (no null
/// termination is required then).
///
/// The variables name must not contain the following special characters:
/// `*`, `/`, `[`, `]`.
///
/// @param[in] variable Pointer to variable
/// @param[in] name Name (null-terminated string, only first 256 characters are taken)
///
/// @returns Error code
/// @retval EVDS_OK Successfully completed
/// @retval EVDS_ERROR_BAD_PARAMETER "variable" is null
/// @retval EVDS_ERROR_BAD_PARAMETER "name" is null
/// @retval EVDS_ERROR_BAD_STATE Object was already initialized
/// @retval EVDS_ERROR_INVALID_OBJECT Object was destroyed
/// @retval EVDS_ERROR_INTERTHREAD_CALL The function can only be called from thread that is initializing the object 
///  (or thread that has created the object before initializer was called)
////////////////////////////////////////////////////////////////////////////////
int EVDS_Variable_SetName(EVDS_VARIABLE* variable, const char* name) {
	int count;
	char clean_name[64];
	char* clean_name_ptr;
	if (!variable) return EVDS_ERROR_BAD_PARAMETER;
	if (!name) return EVDS_ERROR_BAD_PARAMETER;
	if (variable->object && variable->object->initialized) return EVDS_ERROR_BAD_STATE;
#ifndef EVDS_SINGLETHREADED
	if (variable->object) {
		if (variable->object->destroyed) return EVDS_ERROR_INVALID_OBJECT;
		if ((variable->object->create_thread != SIMC_Thread_GetUniqueID()) &&
			(variable->object->initialize_thread != SIMC_Thread_GetUniqueID())) return EVDS_ERROR_INTERTHREAD_CALL;
	}
#endif

	//Sanitize the name
	clean_name_ptr = clean_name;
	for (count = 1; (count <= 64) && (*name); 
		count++, name++, clean_name_ptr++) {
		switch (*name) {
			case '*':
			case '/':
			case '[':
			case ']':
				*clean_name_ptr = '_';
			break;
			default:
				*clean_name_ptr = *name;
			break;
		}
	}
	if (count < 64) *clean_name_ptr = '\0';

	//Store it
	strncpy(variable->name,clean_name,64);
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Get a variables type. @evds_limited_init
///
/// @param[in] variable Variable, type of which will be returned
/// @param[out] type Pointer to variables type
///
/// @returns Error code
/// @retval EVDS_OK Successfully completed
/// @retval EVDS_ERROR_BAD_PARAMETER "variable" is null
/// @retval EVDS_ERROR_BAD_PARAMETER "name" is null
/// @retval EVDS_ERROR_INVALID_OBJECT Object was destroyed
/// @retval EVDS_ERROR_INTERTHREAD_CALL The function can only be called from thread that is initializing the object 
///  (or thread that has created the object before initializer was called)
////////////////////////////////////////////////////////////////////////////////
int EVDS_Variable_GetType(EVDS_VARIABLE* variable, EVDS_VARIABLE_TYPE* type) {
	if (!variable) return EVDS_ERROR_BAD_PARAMETER;
	if (!type) return EVDS_ERROR_BAD_PARAMETER;
#ifndef EVDS_SINGLETHREADED
	if (variable->object) {
		if (variable->object->destroyed) return EVDS_ERROR_INVALID_OBJECT;
		if (!variable->object->initialized &&
			(variable->object->create_thread != SIMC_Thread_GetUniqueID()) &&
			(variable->object->initialize_thread != SIMC_Thread_GetUniqueID())) return EVDS_ERROR_INTERTHREAD_CALL;
	}
#endif

	*type = variable->type;
	return EVDS_OK;
}




////////////////////////////////////////////////////////////////////////////////
/// @brief Get a list of nested variables. @evds_limited_init
///
/// @param[in] variable Variable
/// @param[out] p_list Pointer to list of variables will be written here
///
/// @returns Error code
/// @retval EVDS_OK Successfully completed
/// @retval EVDS_ERROR_BAD_PARAMETER "variable" is null
/// @retval EVDS_ERROR_BAD_PARAMETER "p_list" is null
/// @retval EVDS_ERROR_BAD_STATE Variable type is not EVDS_VARIABLE_TYPE_NESTED
/// @retval EVDS_ERROR_INVALID_OBJECT Object was destroyed
/// @retval EVDS_ERROR_INTERTHREAD_CALL The function can only be called from thread that is initializing the object 
///  (or thread that has created the object before initializer was called)
////////////////////////////////////////////////////////////////////////////////
int EVDS_Variable_GetList(EVDS_VARIABLE* variable, SIMC_LIST** p_list) {
	if (!variable) return EVDS_ERROR_BAD_PARAMETER;
	if (!p_list) return EVDS_ERROR_BAD_PARAMETER;
	if ((variable->type != EVDS_VARIABLE_TYPE_NESTED) &&
		(variable->type != EVDS_VARIABLE_TYPE_FUNCTION)) return EVDS_ERROR_BAD_STATE;
#ifndef EVDS_SINGLETHREADED
	if (variable->object) {
		if (variable->object->destroyed) return EVDS_ERROR_INVALID_OBJECT;
		if (!variable->object->initialized &&
			(variable->object->create_thread != SIMC_Thread_GetUniqueID()) &&
			(variable->object->initialize_thread != SIMC_Thread_GetUniqueID())) return EVDS_ERROR_INTERTHREAD_CALL;
	}
#endif

	*p_list = variable->list;
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Get a list of attributes. @evds_limited_init
///
/// @param[in] variable Variable
/// @param[out] p_list Pointer to list of attributes will be written here
///
/// @returns Error code
/// @retval EVDS_OK Successfully completed
/// @retval EVDS_ERROR_BAD_PARAMETER "variable" is null
/// @retval EVDS_ERROR_BAD_PARAMETER "p_list" is null
/// @retval EVDS_ERROR_BAD_STATE Variable type is not EVDS_VARIABLE_TYPE_NESTED
/// @retval EVDS_ERROR_INVALID_OBJECT Object was destroyed
/// @retval EVDS_ERROR_INTERTHREAD_CALL The function can only be called from thread that is initializing the object 
///  (or thread that has created the object before initializer was called)
////////////////////////////////////////////////////////////////////////////////
int EVDS_Variable_GetAttributes(EVDS_VARIABLE* variable, SIMC_LIST** p_list) {
	if (!variable) return EVDS_ERROR_BAD_PARAMETER;
	if (!p_list) return EVDS_ERROR_BAD_PARAMETER;
	if (variable->type != EVDS_VARIABLE_TYPE_NESTED) return EVDS_ERROR_BAD_STATE;
#ifndef EVDS_SINGLETHREADED
	if (variable->object) {
		if (variable->object->destroyed) return EVDS_ERROR_INVALID_OBJECT;
		if (!variable->object->initialized &&
			(variable->object->create_thread != SIMC_Thread_GetUniqueID()) &&
			(variable->object->initialize_thread != SIMC_Thread_GetUniqueID())) return EVDS_ERROR_INTERTHREAD_CALL;
	}
#endif

	*p_list = variable->attributes;
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Set a floating point (real) value.
///
/// @param[in] variable Variable
/// @param[in] value Value to be set
///
/// @returns Error code
/// @retval EVDS_OK Successfully completed
/// @retval EVDS_ERROR_BAD_PARAMETER "variable" is null
/// @retval EVDS_ERROR_BAD_STATE Variable type invalid (must be EVDS_VARIABLE_TYPE_FLOAT)
/// @retval EVDS_ERROR_INVALID_OBJECT Object was destroyed
////////////////////////////////////////////////////////////////////////////////
int EVDS_Variable_SetReal(EVDS_VARIABLE* variable, EVDS_REAL value) {
	if (!variable) return EVDS_ERROR_BAD_PARAMETER;
	if (variable->type != EVDS_VARIABLE_TYPE_FLOAT) return EVDS_ERROR_BAD_STATE;
#ifndef EVDS_SINGLETHREADED
	if (variable->object && variable->object->destroyed) return EVDS_ERROR_INVALID_OBJECT;
#endif

	*((double*)variable->value) = value;
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Get a floating point (real) value.
///
/// @param[in] variable Variable
/// @param[out] value Pointer to value to be set
///
/// @returns Error code
/// @retval EVDS_OK Successfully completed
/// @retval EVDS_ERROR_BAD_PARAMETER "variable" is null
/// @retval EVDS_ERROR_BAD_PARAMETER "value" is null
/// @retval EVDS_ERROR_BAD_STATE Variable type invalid (must be EVDS_VARIABLE_TYPE_FLOAT)
/// @retval EVDS_ERROR_INVALID_OBJECT Object was destroyed
////////////////////////////////////////////////////////////////////////////////
int EVDS_Variable_GetReal(EVDS_VARIABLE* variable, EVDS_REAL* value) {
	if (!variable) return EVDS_ERROR_BAD_PARAMETER;
	if (!value) return EVDS_ERROR_BAD_PARAMETER;
	if ((variable->type != EVDS_VARIABLE_TYPE_FLOAT) &&
		(variable->type != EVDS_VARIABLE_TYPE_FUNCTION))return EVDS_ERROR_BAD_STATE;
#ifndef EVDS_SINGLETHREADED
	if (variable->object && variable->object->destroyed) return EVDS_ERROR_INVALID_OBJECT;
#endif

	if (variable->type == EVDS_VARIABLE_TYPE_FLOAT) {
		*value = *((double*)variable->value);
	} else {
		*value = ((EVDS_VARIABLE_FUNCTION*)variable->value)->constant_value;
	}
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Set a string value.
///
/// For a nested/arbitrary data variable sets the internal embedded text data.
///
/// @param[in] variable Variable
/// @param[in] value Value to be set
/// @param[in] length Length of string
///
/// @returns Error code
/// @retval EVDS_OK Successfully completed
/// @retval EVDS_ERROR_BAD_PARAMETER "variable" is null
/// @retval EVDS_ERROR_BAD_PARAMETER "value" is null
/// @retval EVDS_ERROR_BAD_STATE Variable type invalid (must be EVDS_VARIABLE_TYPE_STRING)
/// @retval EVDS_ERROR_INVALID_OBJECT Object was destroyed
////////////////////////////////////////////////////////////////////////////////
int EVDS_Variable_SetString(EVDS_VARIABLE* variable, char* value, size_t length) {
	if (!variable) return EVDS_ERROR_BAD_PARAMETER;
	if (!value) return EVDS_ERROR_BAD_PARAMETER;
	if ((variable->type != EVDS_VARIABLE_TYPE_STRING) &&
		(variable->type != EVDS_VARIABLE_TYPE_NESTED)) return EVDS_ERROR_BAD_STATE;
#ifndef EVDS_SINGLETHREADED
	if (variable->object && variable->object->destroyed) return EVDS_ERROR_INVALID_OBJECT;
#endif

#ifndef EVDS_SINGLETHREADED
	SIMC_Lock_Enter(variable->lock);
#endif
	free(variable->value);
	variable->value = (char*)malloc(length*sizeof(char));
	variable->value_size = length;
	strncpy(variable->value,value,length);
#ifndef EVDS_SINGLETHREADED
	SIMC_Lock_Leave(variable->lock);
#endif
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Get a string value.
///
/// For a nested/arbitrary data variable gets the internal embedded text data.
///
/// @param[in] variable Variable
/// @param[out] value Pointer to value to be set. Can be null (then only length is returned)
/// @param[in] max_length Size of the destanation buffers
/// @param[out] length Actual length of string in the variable. Can be null (no length is returned)
///
/// @returns Error code
/// @retval EVDS_OK Successfully completed
/// @retval EVDS_ERROR_BAD_PARAMETER "variable" is null
/// @retval EVDS_ERROR_BAD_STATE Variable type invalid (must be EVDS_VARIABLE_TYPE_STRING)
/// @retval EVDS_ERROR_INVALID_OBJECT Object was destroyed
////////////////////////////////////////////////////////////////////////////////
int EVDS_Variable_GetString(EVDS_VARIABLE* variable, char* value, size_t max_length, size_t* length) {
	if (!variable) return EVDS_ERROR_BAD_PARAMETER;
	if ((variable->type != EVDS_VARIABLE_TYPE_STRING) &&
		(variable->type != EVDS_VARIABLE_TYPE_NESTED)) return EVDS_ERROR_BAD_STATE;
#ifndef EVDS_SINGLETHREADED
	if (variable->object && variable->object->destroyed) return EVDS_ERROR_INVALID_OBJECT;
#endif

#ifndef EVDS_SINGLETHREADED
	SIMC_Lock_Enter(variable->lock);
#endif
	if (length) *length = variable->value_size;
	if (value) strncpy(value,variable->value,(variable->value_size <= max_length ? variable->value_size : max_length));
#ifndef EVDS_SINGLETHREADED
	SIMC_Lock_Leave(variable->lock);
#endif
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Get a copy of a vector value
///
/// @param[in] variable Variable
/// @param[out] value Pointer to value to be set
///
/// @returns Error code
/// @retval EVDS_OK Successfully completed
/// @retval EVDS_ERROR_BAD_PARAMETER "variable" is null
/// @retval EVDS_ERROR_BAD_PARAMETER "value" is null
/// @retval EVDS_ERROR_BAD_STATE Variable type invalid (must be EVDS_VARIABLE_TYPE_VECTOR)
/// @retval EVDS_ERROR_INVALID_OBJECT Object was destroyed
////////////////////////////////////////////////////////////////////////////////
int EVDS_Variable_GetVector(EVDS_VARIABLE* variable, EVDS_VECTOR* value) {
	if (!variable) return EVDS_ERROR_BAD_PARAMETER;
	if (!value) return EVDS_ERROR_BAD_PARAMETER;
	if (variable->type != EVDS_VARIABLE_TYPE_VECTOR) return EVDS_ERROR_BAD_STATE;
#ifndef EVDS_SINGLETHREADED
	if (variable->object && variable->object->destroyed) return EVDS_ERROR_INVALID_OBJECT;
#endif

	memcpy(value,(EVDS_VECTOR*)variable->value,sizeof(EVDS_VECTOR));
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Get a copy of a quaternion value
///
/// @param[in] variable Variable
/// @param[out] value Pointer to value to be set
///
/// @returns Error code
/// @retval EVDS_OK Successfully completed
/// @retval EVDS_ERROR_BAD_PARAMETER "variable" is null
/// @retval EVDS_ERROR_BAD_PARAMETER "value" is null
/// @retval EVDS_ERROR_BAD_STATE Variable type invalid (must be EVDS_VARIABLE_TYPE_QUATERNION)
/// @retval EVDS_ERROR_INVALID_OBJECT Object was destroyed
////////////////////////////////////////////////////////////////////////////////
int EVDS_Variable_GetQuaternion(EVDS_VARIABLE* variable, EVDS_QUATERNION* value) {
	if (!variable) return EVDS_ERROR_BAD_PARAMETER;
	if (!value) return EVDS_ERROR_BAD_PARAMETER;
	if (variable->type != EVDS_VARIABLE_TYPE_QUATERNION) return EVDS_ERROR_BAD_STATE;
#ifndef EVDS_SINGLETHREADED
	if (variable->object && variable->object->destroyed) return EVDS_ERROR_INVALID_OBJECT;
#endif

	memcpy(value,(EVDS_QUATERNION*)variable->value,sizeof(EVDS_QUATERNION));
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Set userdata pointer.
///
/// @param[in] variable Variable
/// @param[in] userdata Pointer to userdata
///
/// @returns Error code
/// @retval EVDS_OK Successfully completed
/// @retval EVDS_ERROR_BAD_PARAMETER "variable" is null
////////////////////////////////////////////////////////////////////////////////
int EVDS_Variable_SetUserdata(EVDS_VARIABLE* variable, void* userdata) {
	if (!variable) return EVDS_ERROR_BAD_PARAMETER;
	variable->userdata = userdata;
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Get userdata pointer.
///
/// @param[in] variable Variable
/// @param[out] p_userdata Pointer to userdata will be written here
///
/// @returns Error code, pointer to userdata
/// @retval EVDS_OK Successfully completed 
/// @retval EVDS_ERROR_BAD_PARAMETER "variable" is null
/// @retval EVDS_ERROR_BAD_PARAMETER "userdata" is null
////////////////////////////////////////////////////////////////////////////////
int EVDS_Variable_GetUserdata(EVDS_VARIABLE* variable, void** p_userdata) {
	if (!variable) return EVDS_ERROR_BAD_PARAMETER;
	if (!p_userdata) return EVDS_ERROR_BAD_PARAMETER;
	*p_userdata = variable->userdata;
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Set a vector value.
///
/// @param[in] variable Variable
/// @param[in] value Value to be set
///
/// @returns Error code
/// @retval EVDS_OK Successfully completed
/// @retval EVDS_ERROR_BAD_PARAMETER "variable" is null
/// @retval EVDS_ERROR_BAD_PARAMETER "value" is null
/// @retval EVDS_ERROR_BAD_STATE Variable type invalid (must be EVDS_VARIABLE_TYPE_VECTOR)
/// @retval EVDS_ERROR_INVALID_OBJECT Object was destroyed
////////////////////////////////////////////////////////////////////////////////
int EVDS_Variable_SetVector(EVDS_VARIABLE* variable, EVDS_VECTOR* value) {
	if (!variable) return EVDS_ERROR_BAD_PARAMETER;
	if (!value) return EVDS_ERROR_BAD_PARAMETER;
	if (variable->type != EVDS_VARIABLE_TYPE_VECTOR) return EVDS_ERROR_BAD_STATE;
#ifndef EVDS_SINGLETHREADED
	if (variable->object && variable->object->destroyed) return EVDS_ERROR_INVALID_OBJECT;
#endif

	memcpy((EVDS_VECTOR*)variable->value,value,sizeof(EVDS_VECTOR));
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Set a quaternion value.
///
/// @param[in] variable Variable
/// @param[in] value Value to be set
///
/// @returns Error code
/// @retval EVDS_OK Successfully completed
/// @retval EVDS_ERROR_BAD_PARAMETER "variable" is null
/// @retval EVDS_ERROR_BAD_PARAMETER "value" is null
/// @retval EVDS_ERROR_BAD_STATE Variable type invalid (must be EVDS_VARIABLE_TYPE_QUATERNION)
/// @retval EVDS_ERROR_INVALID_OBJECT Object was destroyed
////////////////////////////////////////////////////////////////////////////////
int EVDS_Variable_SetQuaternion(EVDS_VARIABLE* variable, EVDS_QUATERNION* value) {
	if (!variable) return EVDS_ERROR_BAD_PARAMETER;
	if (!value) return EVDS_ERROR_BAD_PARAMETER;
	if (variable->type != EVDS_VARIABLE_TYPE_QUATERNION) return EVDS_ERROR_BAD_STATE;
#ifndef EVDS_SINGLETHREADED
	if (variable->object && variable->object->destroyed) return EVDS_ERROR_INVALID_OBJECT;
#endif

	memcpy((EVDS_QUATERNION*)variable->value,value,sizeof(EVDS_QUATERNION));
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Set pointer to custom data
///
/// @param[in] variable Variable
/// @param[in] data Pointer
///
/// @returns Error code
/// @retval EVDS_OK Successfully completed
/// @retval EVDS_ERROR_BAD_STATE Variable type invalid (must be EVDS_VARIABLE_TYPE_DATA_PTR)
/// @retval EVDS_ERROR_BAD_PARAMETER "variable" is null
////////////////////////////////////////////////////////////////////////////////
int EVDS_Variable_SetDataPointer(EVDS_VARIABLE* variable, void* data) {
	if (!variable) return EVDS_ERROR_BAD_PARAMETER;
	if (variable->type != EVDS_VARIABLE_TYPE_DATA_PTR) return EVDS_ERROR_BAD_STATE;
#ifndef EVDS_SINGLETHREADED
	if (variable->object && variable->object->destroyed) return EVDS_ERROR_INVALID_OBJECT;
#endif
	variable->value = data;
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Get pointer to custom data
///
/// @param[in] variable Variable
/// @param[out] data Pointer to data will be written here
///
/// @returns Error code, pointer to userdata
/// @retval EVDS_OK Successfully completed
/// @retval EVDS_ERROR_BAD_STATE Variable type invalid (must be EVDS_VARIABLE_TYPE_DATA_PTR)
/// @retval EVDS_ERROR_BAD_PARAMETER "variable" is null
/// @retval EVDS_ERROR_BAD_PARAMETER "data" is null
////////////////////////////////////////////////////////////////////////////////
int EVDS_Variable_GetDataPointer(EVDS_VARIABLE* variable, void** data) {
	if (!variable) return EVDS_ERROR_BAD_PARAMETER;
	if (!data) return EVDS_ERROR_BAD_PARAMETER;
	if (variable->type != EVDS_VARIABLE_TYPE_DATA_PTR) return EVDS_ERROR_BAD_STATE;
#ifndef EVDS_SINGLETHREADED
	if (variable->object && variable->object->destroyed) return EVDS_ERROR_INVALID_OBJECT;
#endif
	*data = variable->value;
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Set pointer to a function pointer.
///
/// Function signature and meaning is defined by solver that will make use of this function.
/// For example for "gravitational_field" function pointer of the "planet"-type object, the
/// EVDS_Callback_GetGravitationalField function signature must be used.
///
/// @param[in] variable Variable
/// @param[in] data Pointer
///
/// @returns Error code
/// @retval EVDS_OK Successfully completed
/// @retval EVDS_ERROR_BAD_STATE Variable type invalid (must be EVDS_VARIABLE_TYPE_FUNCTION_PTR)
/// @retval EVDS_ERROR_BAD_PARAMETER "variable" is null
////////////////////////////////////////////////////////////////////////////////
int EVDS_Variable_SetFunctionPointer(EVDS_VARIABLE* variable, void* data) {
	if (!variable) return EVDS_ERROR_BAD_PARAMETER;
	if (variable->type != EVDS_VARIABLE_TYPE_FUNCTION_PTR) return EVDS_ERROR_BAD_STATE;
#ifndef EVDS_SINGLETHREADED
	if (variable->object && variable->object->destroyed) return EVDS_ERROR_INVALID_OBJECT;
#endif
	variable->value = data;
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Get pointer to a function pointer.
///
/// @param[in] variable Variable
/// @param[out] data Pointer to data will be written here
///
/// @returns Error code, pointer to userdata
/// @retval EVDS_OK Successfully completed
/// @retval EVDS_ERROR_BAD_STATE Variable type invalid (must be EVDS_VARIABLE_TYPE_FUNCTION_PTR)
/// @retval EVDS_ERROR_BAD_PARAMETER "variable" is null
/// @retval EVDS_ERROR_BAD_PARAMETER "data" is null
////////////////////////////////////////////////////////////////////////////////
int EVDS_Variable_GetFunctionPointer(EVDS_VARIABLE* variable, void** data) {
	if (!variable) return EVDS_ERROR_BAD_PARAMETER;
	if (!data) return EVDS_ERROR_BAD_PARAMETER;
	if (variable->type != EVDS_VARIABLE_TYPE_FUNCTION_PTR) return EVDS_ERROR_BAD_STATE;
#ifndef EVDS_SINGLETHREADED
	if (variable->object && variable->object->destroyed) return EVDS_ERROR_INVALID_OBJECT;
#endif
	*data = variable->value;
	return EVDS_OK;
}