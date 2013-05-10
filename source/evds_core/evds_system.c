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
	if (version_string) strcpy(version_string,EVDS_VERSION_STRING);
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Remove destroyed objects from memory (multithreading version only).
///
/// @evds_mt Will delete objects that are not referenced anywhere, after they have been
/// destroyed with EVDS_Object_Destroy(). The function will physically delete data for
/// the deleted objects with no references.
///
/// @evds_st Nothing is done, returns EVDS_OK.
///
/// The cleanup call can be called in its own dedicated thread, as long as objects
/// are not used after they have been destroyed (or marked as stored with
/// EVDS_Object_Store()).
///
/// Objects which are still being initialized will not be cleaned up until the
/// initialization has finished.
///
/// System will be blocked from being destroyed with EVDS_System_Destroy() until
/// the cleanup call finishes.
///
/// @param[in] system Pointer to system
///
/// @returns Always returns EVDS_OK.
////////////////////////////////////////////////////////////////////////////////
int EVDS_System_CleanupObjects(EVDS_SYSTEM* system) {
#ifndef EVDS_SINGLETHREADED
	int restart;

	//Lock to prevent EVDS_System_Destroy() from deleting objects in another thread
	SIMC_Lock_Enter(system->cleanup_working); 

	//Remove objects until no more objects can be removed
	do {
		SIMC_LIST_ENTRY* entry = SIMC_List_GetFirst(system->deleted_objects);
		restart = 0;

		while (entry) {
			EVDS_OBJECT* object = SIMC_List_GetData(system->deleted_objects,entry);

			//Destroy objects which are not initializing (they have been initialized or the
			// initialization never started), and objects not stored anywhere
			if (((object->initialized == 1) || (object->initialize_thread == SIMC_THREAD_BAD_ID)) && 
				(object->stored_counter == 0)) { 
				//printf("Released object [%p] #%d\n",object,object->uid);

				//Destroy objects data
				EVDS_InternalObject_DestroyData(object);

				//Delete from list and restart
				SIMC_List_Remove(system->deleted_objects,entry); //Stop iterating
				restart = 1;
				break;	
			} else {
				entry = SIMC_List_GetNext(system->deleted_objects,entry);
			}
		}
	} while (restart);

	SIMC_Lock_Leave(system->cleanup_working);
#endif
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Initialize the system and return pointer to a new EVDS_SYSTEM structure.
///
/// EVDS threading system will be initialized with the first EVDS_System_Create call. At
/// least one system must be created to make use of threading functions.
///
/// Example
/// ~~~{.c}
///		EVDS_SYSTEM* system;
///		EVDS_System_Create(&system);
/// ~~~
///
/// @param[out] p_system A pointer to new data structure will be written here
///
/// @returns Error code, pointer to system object
/// @retval EVDS_OK Everything created successfully
/// @retval EVDS_ERROR_BAD_PARAMETER "p_system" is null
/// @retval EVDS_ERROR_MEMORY Error while allocating a data structure
////////////////////////////////////////////////////////////////////////////////
int EVDS_System_Create(EVDS_SYSTEM** p_system)
{
	EVDS_OBJECT* inertial_space;
	EVDS_SYSTEM* system;
	if (!p_system) return EVDS_ERROR_BAD_PARAMETER;

	//Create new system
	system = (EVDS_SYSTEM*)malloc(sizeof(EVDS_SYSTEM));
	*p_system = system;
	if (!system) return EVDS_ERROR_MEMORY;
	memset(system,0,sizeof(EVDS_SYSTEM));

	//Initialize threading and locks
#ifndef EVDS_SINGLETHREADED
	SIMC_Thread_Initialize();
	SIMC_List_Create(&system->deleted_objects,1);
	system->cleanup_working = SIMC_Lock_Create();
#endif

	//Set system to realtime by default
	system->time = EVDS_REALTIME;

	//Data structures
	SIMC_List_Create(&system->object_types,1);
	SIMC_List_Create(&system->objects,1);
	SIMC_List_Create(&system->solvers,1); //FIXME
	SIMC_List_Create(&system->databases,1);

	//Create root inertial space
	EVDS_Object_Create(system,0,&inertial_space);
	EVDS_Object_Initialize(inertial_space,1);
	system->inertial_space = inertial_space;
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Destroy/deinitialize the system.
///
/// Threading system will be deinitialized with the last EVDS_System_Destroy() call.
/// All threads which still do work on objects, related to this system must be stopped
/// before system is destroyed.
///
/// If any API calls are made to objects that belonged to the system destroyed, or
/// API calls are made to the system itself, their result is undefined (most likely
/// causing an application crash).
///
/// All solvers will be unloaded by calling their "OnShutdown" callbacks. The pointer to
/// the system data structure will be removed from memory as well.
///
/// This call can be blocked by EVDS_System_CleanupObjects() until the cleanup
/// operation completes.
///
/// @param[out] system System object to be destroyed
///
/// @returns Error code, pointer to system object
/// @retval EVDS_OK Everything created successfully
/// @retval EVDS_ERROR_BAD_PARAMETER "system" is null
////////////////////////////////////////////////////////////////////////////////
int EVDS_System_Destroy(EVDS_SYSTEM* system)
{
	SIMC_LIST_ENTRY* entry;
	if (!system) return EVDS_ERROR_BAD_PARAMETER;

	//Destroy all objects
#ifndef EVDS_SINGLETHREADED
	SIMC_Lock_Enter(system->cleanup_working); //Make sure cleanup is not running
#endif

	//Remove objects pending for deleting
	EVDS_System_CleanupObjects(system);

	//Deinitialize all solvers
	entry = system->solvers->first;
	while (entry) {
		EVDS_SOLVER* solver = entry->data;
		if (solver->OnShutdown) solver->OnShutdown(system,solver);
		entry = entry->next;
	}

	//Clear out all objects still present
	entry = system->objects->first;
	while (entry) {
		EVDS_OBJECT* object = entry->data;
#ifndef EVDS_SINGLETHREADED
		if (object->initialize_thread != SIMC_THREAD_BAD_ID) {
			SIMC_Thread_Kill(object->initialize_thread); //FIXME: this potentially wrecks everything
		}
#endif
		EVDS_InternalObject_DestroyData(object);
		entry = entry->next;
	}

	//Remove locks
#ifndef EVDS_SINGLETHREADED
	SIMC_Lock_Leave(system->cleanup_working);
	SIMC_Lock_Destroy(system->cleanup_working);
	SIMC_List_Destroy(system->deleted_objects);
#endif

	//Clean up lookup tables
	entry = system->object_types->first;
	while (entry) {
		SIMC_List_Destroy(((EVDS_INTERNAL_TYPE_ENTRY*)entry->data)->objects);
		free(entry->data);
		entry = entry->next;
	}

	//Clean up materials database
	entry = system->databases->first;
	while (entry) {
		//free(entry->data); FIXME
		entry = entry->next;
	}

	//Data structures
	SIMC_List_Destroy(system->object_types);
	SIMC_List_Destroy(system->objects);
	SIMC_List_Destroy(system->solvers);
	SIMC_List_Destroy(system->databases);

	//Remove system data structure and deinitialize threading
#ifndef EVDS_SINGLETHREADED
	SIMC_Thread_Deinitialize();
#endif
	free(system);
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Set system global time (in MJD).
////////////////////////////////////////////////////////////////////////////////
int EVDS_System_SetTime(EVDS_SYSTEM* system, EVDS_REAL time) {
	if (!system) return EVDS_ERROR_BAD_PARAMETER;
	system->time = time;
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Get system global time (in MJD).
////////////////////////////////////////////////////////////////////////////////
int EVDS_System_GetTime(EVDS_SYSTEM* system, EVDS_REAL* time) {
	if (!system) return EVDS_ERROR_BAD_PARAMETER;
	if (!time) return EVDS_ERROR_BAD_PARAMETER;

	//Return real time or system time
	if (system->time == EVDS_REALTIME) {
		*time = SIMC_Thread_GetMJDTime();
	} else {
		*time = system->time;
	}
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Register a new solver.
///
/// A pointer to a structure describing the solver must be passed into this call to
/// register a new solver. This structure must not be destroyed until the system is.
///
/// The solver will be started up and its "OnStartup" callback will be called.
///
/// Short example on defining a solver:
/// ~~~{.c}
///		int EVDS_Solver_Test_Solve(EVDS_SYSTEM* system, EVDS_SOLVER* solver, EVDS_OBJECT* object, EVDS_REAL delta_time);
///		int EVDS_Solver_Test_Integrate(EVDS_SYSTEM* system, EVDS_SOLVER* solver, EVDS_OBJECT* object,
///									   EVDS_REAL delta_time, EVDS_STATE_VECTOR* state, EVDS_STATE_VECTOR_DERIVATIVE* derivative;
///		int EVDS_Solver_Test_Initialize(EVDS_SYSTEM* system, EVDS_SOLVER* solver, EVDS_OBJECT* object) {
///			if (EVDS_Object_CheckType(object,"test_type") != EVDS_OK) return EVDS_IGNORE_OBJECT; 
///			return EVDS_CLAIM_OBJECT;
///		}
///		int EVDS_Solver_Test_Deinitialize(EVDS_SYSTEM* system, EVDS_SOLVER* solver, EVDS_OBJECT* object);
///		
///		EVDS_SOLVER EVDS_Solver_Test = {
///			EVDS_Solver_Test_Initialize, //OnInitialize
///			EVDS_Solver_Test_Deinitialize, //OnDeinitialize
///			EVDS_Solver_Test_Solve, //OnSolve
///			EVDS_Solver_Test_Integrate, //OnIntegrate
///			0, //OnStateSave
///			0, //OnStateLoad
///			0, //OnStartup
///			0, //OnShutdown
///		};
///		
///		EVDS_Solver_Register(system,&EVDS_Solver_Test);
/// ~~~
///
/// Solvers must be registered before the relevant objects are initialized. The API
/// allows adding extra solvers at any time, but in multithreading environment
/// solvers must only be added when no objects are being initialized at the moment.
///
/// If an object is initialized before a recently added solver finishes starting up,
/// the library will enter an undefined state.
///
/// @param[in] system Pointer to system
/// @param[in] solver Pointer to an EVDS_SOLVER structure
///
/// @returns Error code
/// @retval EVDS_OK Successfully completed
/// @retval EVDS_ERROR_BAD_PARAMETER "system" is null
/// @retval EVDS_ERROR_BAD_PARAMETER "solver" is null
////////////////////////////////////////////////////////////////////////////////
int EVDS_Solver_Register(EVDS_SYSTEM* system, EVDS_SOLVER* solver) {
	if (!system) return EVDS_ERROR_BAD_PARAMETER;
	if (!solver) return EVDS_ERROR_BAD_PARAMETER;

	SIMC_List_Append(system->solvers,solver);
	if (solver->OnStartup) solver->OnStartup(system,solver);

	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Get list of initialized objects by type.
///
/// This function returns a list of initialized objects with the given type. Only
/// objects which have been initialized will be listed.
///
/// An empty type list will be returned if the requested type does not exist.
///
/// @param[in] system Pointer to system
/// @param[in] type A null-terminated string (only first 256 characters will be used)
/// @param[out] p_list Pointer to list of objects by type will be written here
///
/// @returns Error code
/// @retval EVDS_OK Successfully completed
/// @retval EVDS_ERROR_BAD_PARAMETER "system" is null
/// @retval EVDS_ERROR_BAD_PARAMETER "type" is null
/// @retval EVDS_ERROR_BAD_PARAMETER "p_list" is null
////////////////////////////////////////////////////////////////////////////////
int EVDS_System_GetObjectsByType(EVDS_SYSTEM* system, const char* type, SIMC_LIST** p_list) {
	SIMC_LIST_ENTRY* entry;
	EVDS_INTERNAL_TYPE_ENTRY* data;
	if (!system) return EVDS_ERROR_BAD_PARAMETER;
	if (!type) return EVDS_ERROR_BAD_PARAMETER;
	if (!p_list) return EVDS_ERROR_BAD_PARAMETER;

	//Search existing object types
	entry = SIMC_List_GetFirst(system->object_types);
	while (entry) {
		data = (EVDS_INTERNAL_TYPE_ENTRY*)SIMC_List_GetData(system->object_types,entry);
		if (strncmp(type,data->type,256) == 0) { //Add to existing list
			*p_list = data->objects;
			SIMC_List_Stop(system->object_types,entry);
			return EVDS_OK;
		}
		entry = SIMC_List_GetNext(system->object_types,entry);
	}

	//Create new object type list
	data = (EVDS_INTERNAL_TYPE_ENTRY*)malloc(sizeof(EVDS_INTERNAL_TYPE_ENTRY));
	strncpy(data->type,type,256); //Set parameters for this entry
	SIMC_List_Create(&data->objects,1);

	//Add to known object types
	SIMC_List_Append(system->object_types,data);
	*p_list = data->objects;
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Get list of all databases (see EVDS_SYSTEM).
///
/// @param[in] system Pointer to system
/// @param[out] p_list Pointer to list of databases will be written here
///
/// @returns Error code
/// @retval EVDS_OK Successfully completed
/// @retval EVDS_ERROR_BAD_PARAMETER "system" is null
/// @retval EVDS_ERROR_BAD_PARAMETER "p_list" is null
////////////////////////////////////////////////////////////////////////////////
int EVDS_System_GetDatabasesList(EVDS_SYSTEM* system, SIMC_LIST** p_list) {
	if (!system) return EVDS_ERROR_BAD_PARAMETER;
	if (!p_list) return EVDS_ERROR_BAD_PARAMETER;

	*p_list = system->databases;
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Set userdata pointer.
///
/// @param[in] system Pointer to system
/// @param[in] userdata Pointer to userdata
///
/// @returns Error code
/// @retval EVDS_OK Successfully completed
/// @retval EVDS_ERROR_BAD_PARAMETER "system" is null
////////////////////////////////////////////////////////////////////////////////
int EVDS_System_SetUserdata(EVDS_SYSTEM* system, void* userdata) {
	if (!system) return EVDS_ERROR_BAD_PARAMETER;
	system->userdata = userdata;
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Get userdata pointer.
///
/// @param[in] system Pointer to system
/// @param[out] p_userdata Pointer to userdata will be written here
///
/// @returns Error code
/// @retval EVDS_OK Successfully completed
/// @retval EVDS_ERROR_BAD_PARAMETER "system" is null
/// @retval EVDS_ERROR_BAD_PARAMETER "userdata" is null
////////////////////////////////////////////////////////////////////////////////
int EVDS_System_GetUserdata(EVDS_SYSTEM* system, void** p_userdata) {
	if (!system) return EVDS_ERROR_BAD_PARAMETER;
	if (!p_userdata) return EVDS_ERROR_BAD_PARAMETER;
	*p_userdata = system->userdata;
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Get object by UID.
///
/// If no parent object is specified, list of all objects in the system will be traversed.
/// Otherwise a recursive search through objects will be initiated.
///
/// This function may return objects which have not yet been initialized. If search returns more
/// than one object, only the first one will be returned by this function.
///
/// @param[in] system Pointer to system
/// @param[in] uid Unique identifier to search for
/// @param[in] parent Object inside which search must be performed (can be null, then all objects are checked)
/// @param[out] p_object Pointer to the object will be written here
///
/// @returns Error code
/// @retval EVDS_OK Successfully completed
/// @retval EVDS_ERROR_BAD_PARAMETER "system" is null
/// @retval EVDS_ERROR_BAD_PARAMETER "p_object" is null
/// @retval EVDS_ERROR_NOT_FOUND No object with this UID was found
////////////////////////////////////////////////////////////////////////////////
int EVDS_System_GetObjectByUID(EVDS_SYSTEM* system, unsigned int uid, EVDS_OBJECT* parent, EVDS_OBJECT** p_object) {
	SIMC_LIST_ENTRY* entry;
	EVDS_OBJECT* child;
	if (!system) return EVDS_ERROR_BAD_PARAMETER;
	if (!p_object) return EVDS_ERROR_BAD_PARAMETER;

	if (parent) { 
		//Traverse parent children
		entry = SIMC_List_GetFirst(parent->raw_children);
		while (entry) {
			child = (EVDS_OBJECT*)SIMC_List_GetData(parent->raw_children,entry);
			//if ((!child->modifier) && (child->uid == uid)) {
			if (child->uid == uid) {
				*p_object = child;
				SIMC_List_Stop(parent->raw_children,entry);
				return EVDS_OK;
			}
			entry = SIMC_List_GetNext(parent->raw_children,entry);
		}

		//If not found amongst children, recursively search inside every child
		entry = SIMC_List_GetFirst(parent->raw_children);
		while (entry) {
			child = (EVDS_OBJECT*)SIMC_List_GetData(parent->raw_children,entry);
			if (EVDS_System_GetObjectByUID(system,uid,child,p_object) == EVDS_OK) { //Found by recursive search
				SIMC_List_Stop(parent->raw_children,entry);
				return EVDS_OK;
			}
			entry = SIMC_List_GetNext(parent->raw_children,entry);
		}
	} else { 
		//Traverse system
		entry = SIMC_List_GetFirst(system->objects);
		while (entry) {
			child = (EVDS_OBJECT*)SIMC_List_GetData(system->objects,entry);
			//if ((!child->modifier) && (child->uid == uid)) {
			if (child->uid == uid) {
				*p_object = child;
				SIMC_List_Stop(system->objects,entry);
				return EVDS_OK;
			}
			entry = SIMC_List_GetNext(system->objects,entry);
		}
	}	
	return EVDS_ERROR_NOT_FOUND;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Get object by name.
///
/// If no parent object is specified, list of all objects in the system will be traversed.
/// Otherwise a recursive search through objects will be initiated.
///
/// This function may return objects which have not yet been initialized. If search returns more
/// than one object, only the first one will be returned by this function.
///
/// @param[in] system Pointer to system
/// @param[in] name Name to search for (null-terminated string, only first 256 characters are taken)
/// @param[in] parent Object inside which search must be performed (can be null, then all objects are checked)
/// @param[out] p_object Pointer to the object will be written here
///
/// @returns Error code
/// @retval EVDS_OK Successfully completed
/// @retval EVDS_ERROR_BAD_PARAMETER "system" is null
/// @retval EVDS_ERROR_BAD_PARAMETER "name" is null
/// @retval EVDS_ERROR_BAD_PARAMETER "p_object" is null
/// @retval EVDS_ERROR_NOT_FOUND No object with this name was found
////////////////////////////////////////////////////////////////////////////////
int EVDS_System_GetObjectByName(EVDS_SYSTEM* system, const char* name, EVDS_OBJECT* parent, EVDS_OBJECT** p_object) {
	SIMC_LIST_ENTRY* entry;
	EVDS_OBJECT* child;
	if (!system) return EVDS_ERROR_BAD_PARAMETER;
	if (!name) return EVDS_ERROR_BAD_PARAMETER;
	if (!p_object) return EVDS_ERROR_BAD_PARAMETER;

	if (parent) { 
		//Traverse parent children
		entry = SIMC_List_GetFirst(parent->raw_children);
		while (entry) {
			child = (EVDS_OBJECT*)SIMC_List_GetData(parent->raw_children,entry);
			//if ((!child->modifier) && (strncmp(child->name,name,256) == 0)) {
			if (strncmp(child->name,name,256) == 0) {
				*p_object = child;
				SIMC_List_Stop(parent->raw_children,entry);
				return EVDS_OK;
			}
			entry = SIMC_List_GetNext(parent->raw_children,entry);
		}

		//If not found amongst children, recursively search inside every child
		entry = SIMC_List_GetFirst(parent->raw_children);
		while (entry) {
			child = (EVDS_OBJECT*)SIMC_List_GetData(parent->raw_children,entry);
			if (EVDS_System_GetObjectByName(system,name,child,p_object) == EVDS_OK) { //Found by recursive search
				SIMC_List_Stop(parent->raw_children,entry);
				return EVDS_OK;
			}
			entry = SIMC_List_GetNext(parent->raw_children,entry);
		}
	} else { 
		//Traverse system
		entry = SIMC_List_GetFirst(system->objects);
		while (entry) {
			child = (EVDS_OBJECT*)SIMC_List_GetData(system->objects,entry);
			//if ((!child->modifier) && (strncmp(child->name,name,256) == 0)) {
			if (strncmp(child->name,name,256) == 0) {
				*p_object = child;
				SIMC_List_Stop(system->objects,entry);
				return EVDS_OK;
			}
			entry = SIMC_List_GetNext(system->objects,entry);
		}
	}	
	return EVDS_ERROR_NOT_FOUND;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Get root inertial space object.
///
/// Returns object that represents the inertial space of the EVDS universe. This object
/// has no state vector defined and all other objects are its children.
///
/// @param[in] system Pointer to system
/// @param[out] p_object Pointer to the root object will be written here
///
/// @returns Error code
/// @retval EVDS_OK Successfully completed
/// @retval EVDS_ERROR_BAD_PARAMETER "system" is null
/// @retval EVDS_ERROR_BAD_PARAMETER "p_object" is null
////////////////////////////////////////////////////////////////////////////////
int EVDS_System_GetRootInertialSpace(EVDS_SYSTEM* system, EVDS_OBJECT** p_object) {
	if (!system) return EVDS_ERROR_BAD_PARAMETER;
	if (!p_object) return EVDS_ERROR_BAD_PARAMETER;

	*p_object = system->inertial_space;
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


////////////////////////////////////////////////////////////////////////////////
/// @brief Load database from a file
////////////////////////////////////////////////////////////////////////////////
int EVDS_System_DatabaseFromFile(EVDS_SYSTEM* system, const char* filename) {
	EVDS_OBJECT proxy_object = { 0 };
	EVDS_OBJECT_LOADEX info = { 0 };
	info.flags = EVDS_OBJECT_LOADEX_NO_OBJECTS;// | EVDS_OBJECT_LOADEX_NO_MODIFIERS;
	
	proxy_object.system = system;
	return EVDS_Object_LoadEx(&proxy_object,filename,&info);
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Load database from a string
////////////////////////////////////////////////////////////////////////////////
int EVDS_System_DatabaseFromString(EVDS_SYSTEM* system, const char* description) {
	EVDS_OBJECT proxy_object = { 0 };
	EVDS_OBJECT_LOADEX info = { 0 };
	info.flags = EVDS_OBJECT_LOADEX_NO_OBJECTS;// | EVDS_OBJECT_LOADEX_NO_MODIFIERS;
	info.description = (char*)description;
	
	proxy_object.system = system;
	return EVDS_Object_LoadEx(&proxy_object,0,&info);
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Get database by name
////////////////////////////////////////////////////////////////////////////////
int EVDS_System_GetDatabaseByName(EVDS_SYSTEM* system, const char* name, EVDS_VARIABLE** p_database) {
	SIMC_LIST_ENTRY* entry;
	EVDS_VARIABLE* child;
	if (!system) return EVDS_ERROR_BAD_PARAMETER;
	if (!name) return EVDS_ERROR_BAD_PARAMETER;
	if (!p_database) return EVDS_ERROR_BAD_PARAMETER;

	entry = SIMC_List_GetFirst(system->databases);
	while (entry) {
		child = (EVDS_VARIABLE*)SIMC_List_GetData(system->databases,entry);
		if (strncmp(child->name,name,256) == 0) {
			*p_database = child;
			SIMC_List_Stop(system->databases,entry);
			return EVDS_OK;
		}
		entry = SIMC_List_GetNext(system->databases,entry);
	}

	return EVDS_ERROR_NOT_FOUND;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Get list of all entries in database
////////////////////////////////////////////////////////////////////////////////
int EVDS_System_GetDatabaseEntries(EVDS_SYSTEM* system, const char* name, SIMC_LIST** p_list) {
	EVDS_VARIABLE* database;
	EVDS_ERRCHECK(EVDS_System_GetDatabaseByName(system,name,&database));
	return EVDS_Variable_GetList(database,p_list);
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Get an entry from a database by name and entry name
////////////////////////////////////////////////////////////////////////////////
int EVDS_System_QueryDatabase(EVDS_SYSTEM* system, const char* query, EVDS_VARIABLE** p_variable) {
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Get variable by a dataref, starting from the root object.
///
/// @todo FIXME
///
/// Examples of various datarefs:
/// Dataref											| Description
/// ------------------------------------------------|----------------------------------------
/// @c /variable_name								| Variable of the root object
/// @c /object_name/variable_name					| Variable of an object inside root object
/// @c /Earth/mass									| Mass of object "Earth" in root object
/// @c /vessel/geometry.cross_sections/section[5]	| 5th variable named "section" (a cross-section) of a vessel
/// @c /vessel/geometry.cross_sections[5]			| 5th nested variable of a variable named "geometry.cross_sections"
///	@c /vessel/geometry.cross_sections[5]/offset	| Attribute of a cross-section
///
/// The root object must not be null. It can either be a user-created object, or the
/// root inertial space object must be used, for example:
/// ~~~{.c}
///		EVDS_OBJECT* root;
///		EVDS_VARIABLE* variable;
///		EVDS_System_GetRootInertialSpace(system,&root);
///		EVDS_Object_QueryVariable(root,"/object_name/variable_name",&variable);
/// ~~~
///
/// @param[in] root Pointer to the root object
/// @param[in] query Dateref that must be retrieved (null-terminated string)
/// @param[out] p_variable Pointer to variable will be written here
///
/// @returns Error code, pointer to a variable
/// @retval EVDS_OK Successfully completed (object matches type)
/// @retval EVDS_ERROR_BAD_PARAMETER "root" is null
/// @retval EVDS_ERROR_BAD_PARAMETER "query" is null
/// @retval EVDS_ERROR_BAD_PARAMETER "p_variable" is null
//  @retval EVDS_ERROR_NOT_FOUND Could not resolve the dataref
/// @retval EVDS_ERROR_INVALID_OBJECT Root object was destroyed
////////////////////////////////////////////////////////////////////////////////
int EVDS_System_QueryObject(EVDS_OBJECT* root, const char* query, EVDS_VARIABLE** p_variable, EVDS_OBJECT** p_object) {
	EVDS_OBJECT* object = root; //Object, inside which the query is performed
	EVDS_VARIABLE* variable = 0; //Variable, inside which the query is performed
	int in_variable = 0; //Is currently inside a variable

	char *token_start, *token_end; //Parameters for the token word
	size_t token_length;
	if (!root) return EVDS_ERROR_BAD_PARAMETER;
	if (!query) return EVDS_ERROR_BAD_PARAMETER;
	if (!p_variable) return EVDS_ERROR_BAD_PARAMETER;
#ifndef EVDS_SINGLETHREADED
	if (root->destroyed) return EVDS_ERROR_INVALID_OBJECT;
#endif

	//Parse the query
	token_start = (char*)query;
	while (*token_start) {
		//Find next token or fetch entire remaining string
		token_end = strchr(token_start,'/');
		if (token_end) {
			token_length = token_end-token_start;
		} else {
			token_length = strlen(token_start);
		}

		//Parse token if it was found
		if (token_length != 0) {
			SIMC_LIST_ENTRY* entry;

			if (!in_variable) { //Logic for searching variables inside objects
				EVDS_OBJECT* found_object = 0;

				//Check if token is an objects name
				entry = SIMC_List_GetFirst(object->raw_children);
				while (entry) {
					char name[257];
					EVDS_OBJECT* child = SIMC_List_GetData(object->raw_children,entry);
					EVDS_Object_GetName(child,name,256); name[256] = 0;
					if (strncmp(name,token_start,token_length) == 0) {
						found_object = child;
						break;
					}	
					entry = SIMC_List_GetNext(object->raw_children,entry);
				}
				SIMC_List_Stop(object->raw_children,entry);

				//Check if token is a variables name
				if (!found_object) {
					entry = SIMC_List_GetFirst(object->variables);
					while (entry) {
						char name[257];
						EVDS_VARIABLE* child_variable = SIMC_List_GetData(object->variables,entry);
						EVDS_Variable_GetName(child_variable,name,256); name[256] = 0;
						if (strncmp(name,token_start,token_length) == 0) {
							variable = child_variable;
							in_variable = 1;

							if (!token_end) { //Found the correct variable
								*p_variable = variable;
								return EVDS_OK;
							}
							break;
						}	
						entry = SIMC_List_GetNext(object->variables,entry);
					}
					SIMC_List_Stop(object->variables,entry);
				}

				//Move to next object or variable
				if (found_object) {
					object = found_object;
				}
			} else {
				//FIXME
			}
		}

		//Move to next token
		token_start += token_length+1;
	}

	//Query failed
	return EVDS_ERROR_NOT_FOUND;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Set global initialization callback for system.
///
/// This system-wide callback will be called before every object is initialized. It can be used to append
/// additional simulator-specific variables of interest. The callback will be called from the initializing
/// thread (which may be different from the one that called EVDS_Object_Initialize()).
///
/// The system-wide callback must return EVDS_OK if completed successfully. EVDS_CLAIM_OBJECT can be returned
/// to claim the object (the solvers initialization routine will then be ignored).
///
/// Call with null callback pointer to disable.
///
/// @param[in] system Pointer to system
/// @param[in] p_callback Pointer to the callback function (can be null)
///
/// @returns Error code
/// @retval EVDS_OK Successfully completed
/// @retval EVDS_ERROR_BAD_PARAMETER "system" is null
////////////////////////////////////////////////////////////////////////////////
int EVDS_System_SetCallback_OnInitialize(EVDS_SYSTEM* system, EVDS_Callback_Initialize* p_callback) {
	if (!system) return EVDS_ERROR_BAD_PARAMETER;

	system->OnInitialize = p_callback;
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// Signals an assert has failed
////////////////////////////////////////////////////////////////////////////////
int EVDS_AssertFailed(const char* what, const char* filename, int line) {
	printf("Assert failed: %s (%s:%d)\n", what, filename, line);
	return EVDS_OK;
}