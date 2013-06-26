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

#ifdef _WIN32
#	include <windows.h>
#endif


////////////////////////////////////////////////////////////////////////////////
/// @brief Default internal solver. Simply calls EVDS_Object_Solve() for all children.
///
/// The default solver is used when no solver is defined for the object.
////////////////////////////////////////////////////////////////////////////////
int EVDS_InternalCallback_Solve(EVDS_SYSTEM* system, EVDS_SOLVER* solver, EVDS_OBJECT* object, EVDS_REAL delta_time) {
	SIMC_LIST_ENTRY* entry = SIMC_List_GetFirst(object->children);
	while (entry) {
		EVDS_OBJECT* child = (EVDS_OBJECT*)SIMC_List_GetData(object->children,entry);
		EVDS_Object_Solve(child,delta_time);
		entry = SIMC_List_GetNext(object->children,entry);
	}
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Solve object and update state of all its children.
///
/// Runs the solver for the object. Will first attempt to execute solving function that is
/// defined specially for this object (see EVDS_Object_SetCallback_OnSolve()). If no function is defined,
/// solver that has claimed the object will be used (if object has a known type defined).
/// See EVDS_SOLVER::OnSolve for more information on creating a solving function.
///
/// If neither solving function or a solver are available, uses the default solver which calls
/// EVDS_Object_Solve() for every child object.
///
/// Time step \f$\Delta t\f$ is the amount of time by which internal state of the object must be
/// propagated forward. The solver may internally split a single time increment into sub-intervals.
///
/// The solver may update internal state of the object based on its current state vector. If any
/// object state changes rapidly and must be integrated along with the objects state vector, it must
/// be made part of the state vector, see EVDS_STATE_VECTOR.
///
/// @note Time step cannot be negative (only forward state propagation is allowed).
///
/// @param[in] object Object to solve
/// @param[in] delta_time Time step \f$\Delta t\f$ for the solver
///
/// @returns Error code from function or error code from solver
/// @retval EVDS_OK No errors during execution
/// @retval EVDS_ERROR_BAD_PARAMETER "object" pointer is null
/// @retval EVDS_ERROR_NOT_INITIALIZED Object was not initialized (see EVDS_Object_Initialize())
/// @retval EVDS_ERROR_INVALID_OBJECT "object" was already destroyed
/// @retval ... Error code returned from the solvers callback
////////////////////////////////////////////////////////////////////////////////
int EVDS_Object_Solve(EVDS_OBJECT* object, EVDS_REAL delta_time) {
	if (!object) return EVDS_ERROR_BAD_PARAMETER;
	if (!object->initialized) return EVDS_ERROR_NOT_INITIALIZED;
#ifndef EVDS_SINGLETHREADED
	if (object->destroyed) return EVDS_ERROR_INVALID_OBJECT;
#endif

	if (object->solve) {
		return object->solve(object->system,0,object,delta_time);
	} else if (object->solver && (object->solver->OnSolve)) {
		return object->solver->OnSolve(object->system,object->solver,object,delta_time);
	} else {
		return EVDS_InternalCallback_Solve(object->system,0,object,delta_time);
	}
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Compute derivative of the objects state vector at some point in time.
///
/// Returns derivative of the objects state vector as based on the current state vector and
/// the current internal (hidden) state of the object.
///
/// This function is usually used by the propagators to compute accelerations
/// and velocities to compute the motion trajectory. Alternatively it can be used
/// by vessel objects to request forces and torques from children.
///
/// The object may either set the "common" set of variables, or set "torque" and "force"
/// as a return value. The exact use of these variables depends on the parent object.
///
/// If object has a custom integration function defined, it will be called instead of
/// the default solvers integration function. If there is no solver integration function,
/// the corresponding state vector variables will be copied into the derivative:
///  - Acceleration
///  - Velocity
///  - Angular acceleration
///  - Angular velocity
///
/// Example of using this callback for writing a propagator (simple forward integration):
/// ~~~{.c}
///		int EVDS_ForwardIntegration_Solve(EVDS_SYSTEM* system, EVDS_SOLVER* solver, EVDS_OBJECT* coordinate_system, EVDS_REAL delta_time) {
///			SIMC_LIST_ENTRY* entry;
///			SIMC_LIST* list;
///		
///			EVDS_Object_GetChildren(coordinate_system,&list);
///			entry = SIMC_List_GetFirst(list);
///			while (entry) {
///				EVDS_STATE_VECTOR state;
///				EVDS_OBJECT* object = (EVDS_OBJECT*)SIMC_List_GetData(list,entry);
///				EVDS_Object_Solve(object,delta_time);
///
///				//Get current state vector and integrate it
///				EVDS_Object_GetStateVector(object,&state);
///				EVDS_Object_Integrate(object,delta_time,&state,&state_derivative);
///				
///				//Propagate state vector and update it in object
///				EVDS_StateVector_MultiplyByTimeAndAdd(&state,&state,&state_derivative,delta_time);
///				EVDS_Object_SetStateVector(object,&state);
///
///				//Move to next object in list
///				entry = SIMC_List_GetNext(list,entry);
///			}
///			SIMC_List_Stop(list,entry);
/// ~~~
/// 
/// @todo It is not possible to set state per-thread for an object, and therefore not possible to
///  call integration from multiple threads at once. The future EVDS version will change this
///  to allow simultanous calls to integration from many threads.
///
/// @evds_mt This function can only be called from one thread at a single time - only one
///  thread may perform integration of an object at the same time. Calling integration from two threads
///  will put the EVDS system into indeterminate state (there are no protection mechanisms in place).
///
/// @note Time step cannot be negative (only forward state propagation is allowed).
///
/// @param[in] object Object to integrate
/// @param[in] delta_time Time step \f$\Delta t\f$ since state vectors last update time
/// @param[in] state State vector for which derivative must be found
/// @param[out] derivative Derivative of the state vector
///
/// @returns Error code from function or error code from integration
/// @retval EVDS_OK No errors during execution
/// @retval EVDS_ERROR_BAD_PARAMETER "object" pointer is null
/// @retval EVDS_ERROR_NOT_INITIALIZED Object was not initialized (see EVDS_Object_Initialize())
/// @retval EVDS_ERROR_INVALID_OBJECT "object" was already destroyed
////////////////////////////////////////////////////////////////////////////////
int EVDS_Object_Integrate(EVDS_OBJECT* object, EVDS_REAL delta_time, EVDS_STATE_VECTOR* state,
						  EVDS_STATE_VECTOR_DERIVATIVE* derivative) {
	int error_code;
	EVDS_STATE_VECTOR* passed_state;
	if (!object) return EVDS_ERROR_BAD_PARAMETER;
	if (!object->initialized) return EVDS_ERROR_NOT_INITIALIZED;
#ifndef EVDS_SINGLETHREADED
	if (object->destroyed) return EVDS_ERROR_INVALID_OBJECT;
#endif

	//Set integrate thread (so coordinate transformations outside this thread use old public state vector)
	if (state) {
		passed_state = state;
#ifndef EVDS_SINGLETHREADED
		EVDS_InternalObject_SetPrivateStateVector(object,state);
#else
		EVDS_Object_SetStateVector(object,state);
#endif
	} else {
		passed_state = &object->state;
#ifndef EVDS_SINGLETHREADED
		EVDS_InternalObject_SetPrivateStateVector(object,&object->state);
#endif
	}
#ifndef EVDS_SINGLETHREADED
	object->integrate_thread = SIMC_Thread_GetUniqueID();
#endif

	//Initialize derivative
	EVDS_StateVector_Derivative_Initialize(derivative,object->parent);

	//Run integration
	if (object->integrate) {
		error_code = object->integrate(object->system,0,object,delta_time,passed_state,derivative);
	} else if (object->solver && (object->solver->OnIntegrate)) {
		error_code = object->solver->OnIntegrate(object->system,object->solver,object,delta_time,passed_state,derivative);
	} else {
		SIMC_SRW_EnterRead(object->state_lock);
		EVDS_Vector_Copy(&derivative->acceleration,&object->state.acceleration);
		EVDS_Vector_Copy(&derivative->velocity,&object->state.velocity);
		EVDS_Vector_Copy(&derivative->angular_acceleration,&object->state.angular_acceleration);
		EVDS_Vector_Copy(&derivative->angular_velocity,&object->state.angular_velocity);
		SIMC_SRW_LeaveRead(object->state_lock);
	}
#ifndef EVDS_SINGLETHREADED
	object->integrate_thread = SIMC_THREAD_BAD_ID;
#endif
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Calculates moments of inertia/radius of gyration tensor for a body with mass
///
/// Creates \f$R_g^2\f$ tensor from any possible data specified in the object.
/// The following input data is sufficient for determining \f$R_g^2\f$ tensor:
///  - @c ix/iy/iz (inertia tensor)
///  - @c ixx/iyy/izz (principial elements of the tensor)
///  - @c jxx/jyy/jzz (principial elements of the \f$R_g^2\f$ tensor)
///  - No input data (an empty tensor will be created to be later filled out from objects geometry)
///
/// See EVDS_Object_Initialize() for more information.
////////////////////////////////////////////////////////////////////////////////
int EVDS_InternalObject_ComputeMIMatrix(EVDS_OBJECT* object, char axis, EVDS_REAL mass, EVDS_VARIABLE** j_var) {
	//Names for extra variables
	char iname[32],ipname[32]; //Inertia tensor, principial element
	char jname[32],jpname[32]; //Gyration radius squared tensor, principial element

	//Extra variables
	EVDS_VARIABLE *ip, *jp; //Used for a principial element on a diagonal of a matrix
	EVDS_VARIABLE *i, *j; //Used for a vector-row of a matrix
	EVDS_REAL ip_v,jp_v;

	//Temporary variables
	EVDS_VECTOR vector;
	EVDS_Vector_Initialize(vector);
	*j_var = 0;

	//Compute proper names for the variables
	sprintf(iname,"i%c",axis);
	sprintf(ipname,"i%c%c",axis,axis);
	sprintf(jname,"j%c",axis);
	sprintf(jpname,"j%c%c",axis,axis);

	//Check if specified as a principial element (jxx)
	jp_v = 0.0;
	if (EVDS_Object_GetVariable(object,jpname,&jp) == EVDS_OK) {
		EVDS_Variable_GetReal(jp,&jp_v);
	}
	ip_v = 0.0;
	if (EVDS_Object_GetVariable(object,ipname,&ip) == EVDS_OK) {
		EVDS_Variable_GetReal(ip,&ip_v);
	}

	if (jp_v > 0.0) {
		EVDS_Object_AddVariable(object,jname,EVDS_VARIABLE_TYPE_VECTOR,&j); //Add 'jx' matrix row
		switch (axis) {
			case 'x': vector.x = jp_v; break;
			case 'y': vector.y = jp_v; break;
			case 'z': vector.z = jp_v; break;
		}
		vector.coordinate_system = object;
		EVDS_Variable_SetVector(j,&vector);
	} else if (EVDS_Object_GetVariable(object,jname,&j) == EVDS_ERROR_NOT_FOUND) { //Not specified as 'jx' or 'jxx'
		EVDS_Object_AddVariable(object,jname,EVDS_VARIABLE_TYPE_VECTOR,&j);

		//Check if moments of inertia principial value (ixx) is defined
		if (ip_v > 0.0) {
			switch (axis) {
				case 'x': vector.x = ip_v / mass; break;
				case 'y': vector.y = ip_v / mass; break;
				case 'z': vector.z = ip_v / mass; break;
			}
			vector.coordinate_system = object;
			EVDS_Variable_SetVector(j,&vector);
		} else if (EVDS_Object_GetVariable(object,iname,&i) == EVDS_OK) { //Convert from inertia tensor row (ix)
			EVDS_Variable_GetVector(i,&vector);
			vector.x /= mass;
			vector.y /= mass;
			vector.z /= mass;
			vector.coordinate_system = object;
			EVDS_Variable_SetVector(j,&vector);
		} else {
			*j_var = j;
		}
	}
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Calculate mass parameters if applicable.
///
/// The function will compute additional mass parameters for the object if it has
/// 'mass' variable defined, and perform basic sanity checks against input values.
///
/// If object does not have center of mass or inertia tensor defined, they will be
/// calculated from the objects mesh.
///
/// If the object is empty, it will still have a valid but insignificant (under EVDS_EPS)
/// mass and moments of inertia tensor to assure numerical stability.
///
/// See EVDS_Object_Initialize() for more information.
////////////////////////////////////////////////////////////////////////////////
int EVDS_InternalObject_ComputeMassParameters(EVDS_OBJECT* object) {
	EVDS_VARIABLE* v_mass;
	EVDS_VARIABLE* v_cm;
	EVDS_VARIABLE* v_jx;
	EVDS_VARIABLE* v_jy;
	EVDS_VARIABLE* v_jz;
	EVDS_REAL mass;

	//Check if object has mass defined
	EVDS_ERRCHECK(EVDS_Object_GetVariable(object,"mass",&v_mass));
	EVDS_ERRCHECK(EVDS_Variable_GetReal(v_mass,&mass));

	//Mass cannot be negative
	if (mass < EVDS_EPS) {
		mass = EVDS_EPS;
		EVDS_ERRCHECK(EVDS_Variable_SetReal(v_mass,mass));
	}

	//Calculate inertia tensor components
	EVDS_ERRCHECK(EVDS_InternalObject_ComputeMIMatrix(object,'x',mass,&v_jx));
	EVDS_ERRCHECK(EVDS_InternalObject_ComputeMIMatrix(object,'y',mass,&v_jy));
	EVDS_ERRCHECK(EVDS_InternalObject_ComputeMIMatrix(object,'z',mass,&v_jz));

	//Check if center of mass must be determined
	if (EVDS_Object_GetVariable(object,"cm",&v_cm) != EVDS_OK) {
		EVDS_ERRCHECK(EVDS_Object_AddVariable(object,"cm",EVDS_VARIABLE_TYPE_VECTOR,&v_cm));
	} else {
		v_cm = 0;
	}

	//Calculate inertia tensor from geometry
	if (v_jx || v_jy || v_jz || v_cm) { //FIXME: add proper flags to mesh calls
		int i;
		EVDS_MESH* mesh;

		EVDS_VECTOR jx;
		EVDS_VECTOR jy;
		EVDS_VECTOR jz;
		EVDS_VECTOR cm;
		EVDS_Vector_Initialize(jx);
		EVDS_Vector_Initialize(jy);
		EVDS_Vector_Initialize(jz);
		EVDS_Vector_Initialize(cm);

		//Generate adequate quality mesh
		EVDS_ERRCHECK(EVDS_Mesh_Generate(object,&mesh,50.0f,EVDS_MESH_USE_DIVISIONS));

		//Compute center of mass
		cm.coordinate_system = object;
		for (i = 0; i < mesh->num_triangles; i++) {
			float w = mesh->triangles[i].area;

			cm.x += w*mesh->triangles[i].center.x;
			cm.y += w*mesh->triangles[i].center.y;
			cm.z += w*mesh->triangles[i].center.z;
		}
		cm.x /= mesh->total_area + EVDS_EPS;//*((EVDS_REAL)mesh->num_triangles + EVDS_EPS);
		cm.y /= mesh->total_area + EVDS_EPS;//*((EVDS_REAL)mesh->num_triangles + EVDS_EPS);
		cm.z /= mesh->total_area + EVDS_EPS;//*((EVDS_REAL)mesh->num_triangles + EVDS_EPS);		
		EVDS_ERRCHECK(EVDS_Variable_SetVector(v_cm,&cm));

		//Calculate radius of gyration matrix
		for (i = 0; i < mesh->num_triangles; i++) {
			float w = mesh->triangles[i].area;
			float x = (float)(mesh->triangles[i].center.x - cm.x);
			float y = (float)(mesh->triangles[i].center.y - cm.y);
			float z = (float)(mesh->triangles[i].center.z - cm.z);

			jx.x += w*(pow(y,2) + pow(z,2));
			jx.y -= w*x*y;
			jx.z -= w*x*z;

			jy.x -= w*y*x;
			jy.y += w*(pow(x,2) + pow(z,2));
			jy.z -= w*y*z;

			jz.x -= w*z*x;
			jz.y -= w*z*y;
			jz.z += w*(pow(x,2) + pow(y,2));
		}

		//Divide by total mass
		jx.x = jx.x/(mesh->total_area+EVDS_EPSf);
		jy.y = jy.y/(mesh->total_area+EVDS_EPSf);
		jz.z = jz.z/(mesh->total_area+EVDS_EPSf);

		//Store required components
		jx.coordinate_system = object;
		jy.coordinate_system = object;
		jz.coordinate_system = object;
		if (v_jx) EVDS_Variable_SetVector(v_jx,&jx);
		if (v_jy) EVDS_Variable_SetVector(v_jy,&jy);
		if (v_jz) EVDS_Variable_SetVector(v_jz,&jz);

		//Clean up
		EVDS_Mesh_Destroy(mesh);
	}
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Thread that initializes object (can also be called as a routine).
///
/// See EVDS_Object_Initialize() for more information.
////////////////////////////////////////////////////////////////////////////////
void EVDS_InternalThread_Initialize_Object(EVDS_OBJECT* object) {
	EVDS_SYSTEM* system = object->system;
	SIMC_LIST* objects_list;
	SIMC_LIST_ENTRY* entry;

	//Set initialization thread for this object
#ifndef EVDS_SINGLETHREADED
	object->initialize_thread = SIMC_Thread_GetUniqueID();
#endif

	//Make sure all children have unique names FIXME
	//

	//Make sure all children have unique identifiers FIXME
	//

	//Initialize all children
	entry = SIMC_List_GetFirst(object->raw_children);
	while (entry) {
		EVDS_OBJECT* child = (EVDS_OBJECT*)SIMC_List_GetData(object->raw_children,entry);
#ifndef EVDS_SINGLETHREADED
		child->initialize_thread = SIMC_Thread_GetUniqueID();
#endif
		SIMC_List_Stop(object->raw_children,entry); //Stop iterator so initializer can change list of children
		EVDS_InternalThread_Initialize_Object(child); //Blocking initialization
		
		//Find next un-initialized child
		entry = SIMC_List_GetFirst(object->raw_children);
		while (entry) {
			EVDS_OBJECT* child = (EVDS_OBJECT*)SIMC_List_GetData(object->raw_children,entry);
			if (child->initialized) {
				entry = SIMC_List_GetNext(object->raw_children,entry);
			} else {
				break;
			}
		}
	}

	//Check every solver if it wants to claim the object
	entry = SIMC_List_GetFirst(system->solvers);
	while (entry) {
		EVDS_SOLVER* solver = SIMC_List_GetData(system->solvers,entry);

		//Check if this solver will not ignore the object
		int error_code = EVDS_IGNORE_OBJECT;
		if (system->OnInitialize) {
			error_code = system->OnInitialize(system,solver,object);
			if ((error_code == EVDS_OK) && (solver->OnInitialize)) {
				error_code = solver->OnInitialize(system,solver,object);
			}
		} else {
			if (solver->OnInitialize) error_code = solver->OnInitialize(system,solver,object);
		}

		//Initialized successfully?
		if (error_code == EVDS_CLAIM_OBJECT) {
			object->solver = solver;
			SIMC_List_Stop(system->solvers,entry);
			break;
		} else if (error_code != EVDS_IGNORE_OBJECT) { 
			//An error has occured
			EVDS_Object_Destroy(object);
		}
		entry = SIMC_List_GetNext(system->solvers,entry);
	}

	//Initialize rigid body parameters (any objects may have mass/moments of inertia defined)
	EVDS_InternalObject_ComputeMassParameters(object);

	//Finished initializing (might not actually have a solver defined!)
	object->initialized = 1;

	//Add to object-by-type lookup list
	if (EVDS_System_GetObjectsByType(object->system,object->type,&objects_list) == EVDS_OK) {
		object->type_entry = SIMC_List_Append(objects_list,object);
		object->type_list = objects_list;
	}

	//Add to list of parent's children
	if (object->parent) {		
		object->parent_entry = SIMC_List_Append(object->parent->children,object);
	}
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Initialize the object.
///
/// This will finalize the object structure (no new variables may be added after this call).
/// The call can be blocking (in current thread), or a new thread may be created for the object
/// to finish initialization in.
///
/// @note The objects ownership will be transferred to the initializing thread if non-blocking
///       initialization is used. This will prevent the main thread from accessing object data
///       until it finishes initializing. For example, EVDS_Object_GetName() will fail from the main
///       thread until the object is fully initialized.
///
/// Non-blocking initialization is best used for loading objects, which may take a while to initialize,
/// for example if some expensive precomputing is being done for those.
///
/// Example of use:
/// ~~~{.c}
/// 	EVDS_Object_Create(system,0,&inertial_space);
///		EVDS_Object_SetType(inertial_space,"propagator_rk4");
///		EVDS_Object_Initialize(inertial_space,1);
///		//Execution will continue only after object was initialized
/// ~~~
///
/// Before object is initialized the following actions will be executed in the listed order:
///  - If any children have duplicate names (within this object), they will be renamed arbitrarily to unique names.
///  - If any children have unique identifiers which are already used globally, they will be assigned new identifiers.
///  - All children will be initialized in a blocking way.
///
/// @note The list of children will be unlocked while initializing the objects child. Children objects may
///       create additional objects under the object being initialized.
///
/// Every solver registered in the EVDS system will be called to check if the object being initialized must be
/// claimed. If global initialization callback is defined, it will be called first for every solver. See
/// EVDS_System_SetCallback_OnInitialize().
///
/// After object is marked as initialized, it will be added to list of parents children and to list of objects
/// by type.
///
/// If object has a "mass" variable defined, additional mass-related properties will be calculated. If mass
/// is zero or negative, it will be made infinitely small. Objects radius of gyration squared tensor will be calculated from:
///  - @c ix/iy/iz (inertia tensor specified explicitly)
///  - @c jx/jy/jz (radius of gyration squared tensor specified explicitly)
///  - @c ixx/iyy/izz (principial elements of the tensor)
///  - @c jxx/jyy/jzz (principial elements of the radius of gyration squared tensor)
///  - Automatically from objects mesh
///
/// The radius of gyrations squared tensor will be written as vector variables @c jx/jy/jz.
///
/// @note If inertia tensor is not defined for the object explicitly, the tensor variables (@c ix/iy/iz)
///       will not be created. Inertia tensor can be calculated from mass and radius of gyration squared as
///       \f$I_{ij} = R_{g,ij}^2 \cdot mass\f$.
///
/// If object does not have center of mass defined it will also be calculated from the objects mesh
/// using weighted average between all vertices of the mesh.
/// If the object is empty, it will still have a valid but insignificant (under EVDS_EPS)
/// mass and moments of inertia tensor to assure numerical stability.
///
/// @evds_st Always blocking, ignores value of "is_blocking"
///
/// @param[in] object Object to be initialized
/// @param[in] is_blocking Should object block current threads execution with its initialization
///
/// @returns Error code
/// @retval EVDS_OK Successfully completed (does not report state of initialization)
/// @retval EVDS_ERROR_BAD_PARAMETER "object" is null
/// @retval EVDS_ERROR_BAD_STATE Object is already initialized
////////////////////////////////////////////////////////////////////////////////
int EVDS_Object_Initialize(EVDS_OBJECT* object, int is_blocking) {
	if (!object) return EVDS_ERROR_BAD_PARAMETER;
	if (object->initialized) return EVDS_ERROR_BAD_STATE;

#ifndef EVDS_SINGLETHREADED
	if (is_blocking) {
		EVDS_InternalThread_Initialize_Object(object);
	} else {
		object->create_thread = SIMC_THREAD_BAD_ID;
		SIMC_Thread_Create(EVDS_InternalThread_Initialize_Object,object);
	}
#else
	EVDS_InternalThread_Initialize_Object(object);
#endif
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Destroys object from the system.
///
/// @evds_mt Object data will remain in memory until it is no longer referenced anywhere
/// (see EVDS_Object_Store(), EVDS_Object_Release()). The data will only be cleaned when
/// EVDS_System_CleanupObjects() API call is executed.
///
/// Destroyed objects must not be passed to the EVDS API. Any calls using destroyed objects
/// will fail and will enter undefined state if objects data was cleaned up (by
/// EVDS_System_CleanupObjects()).
///
/// After this function is called, object will be removed from the following lists, in this order:
///	 - List of all objects in the system
///	 - List of initialized children of the parent object
///	 - List of all children of the parent object
///	 - List of initialized objects by type
///
/// If any iterator is in progress while an object is removed, the removing thread will be blocked
/// until all iterators complete.
///
/// @note If object is still accessed from inside an iterator, it will be considered valid, but may not be
///       consitently represented in the linked lists above.
///
/// All children of the object will be destroyed after the object itself is no longer valid
/// and is no longer represented in any lists. Finally the solver will deinitialize the object.
/// The object will be then marked as destroyed and the API calls will stop accepting it.
///
/// @evds_st The object data will be removed right away.  The data for the object pointer will be
///  destroyed and the pointer will become invalid. All children will be removed just as in the multithreading case.
///
/// @param[in] object Object to be destroyed
///
/// @returns Always returns successfully
/// @retval EVDS_OK Successfully completed
/// @retval EVDS_ERROR_BAD_PARAMETER "object" pointer is null
/// @retval EVDS_ERROR_INVALID_OBJECT "object" was already destroyed
////////////////////////////////////////////////////////////////////////////////
int EVDS_Object_Destroy(EVDS_OBJECT* object) {
	SIMC_LIST_ENTRY* entry;
	if (!object) return EVDS_ERROR_BAD_PARAMETER;
	if (object->destroyed) return EVDS_ERROR_INVALID_OBJECT;

#ifndef EVDS_SINGLETHREADED
	//Delete object from various lists
	SIMC_List_GetFirst(object->system->objects);
	SIMC_List_Remove(object->system->objects,object->object_entry);
	if (object->parent && object->parent_entry) {
		SIMC_List_GetFirst(object->parent->children);
		SIMC_List_Remove(object->parent->children,object->parent_entry);
	}
	if (object->parent && object->rparent_entry) {
		SIMC_List_GetFirst(object->parent->raw_children);
		SIMC_List_Remove(object->parent->raw_children,object->rparent_entry);
	}
	if (object->type_entry) {
		SIMC_List_GetFirst(object->type_list);
		SIMC_List_Remove(object->type_list,object->type_entry);
	}
#else
	SIMC_List_Remove(object->system->objects,object->object_entry);
	if (object->parent && object->parent_entry) SIMC_List_Remove(object->parent->children,object->parent_entry);
	if (object->parent && object->rparent_entry) SIMC_List_Remove(object->parent->raw_children,object->rparent_entry);
	if (object->type_entry) SIMC_List_Remove(object->type_list,object->type_entry);
#endif

	//Request all children destroyed first (stop iteration so the raw children list will not be locked)
	entry = SIMC_List_GetFirst(object->raw_children);
	while (entry) {
		EVDS_OBJECT* child = (EVDS_OBJECT*)SIMC_List_GetData(object->raw_children,entry);
		SIMC_List_Stop(object->raw_children,entry); //Stop iterating

		if (EVDS_Object_Destroy(child) == EVDS_OK) { //Destroy object and restart iteration
			entry = SIMC_List_GetFirst(object->raw_children);
		} else {
			entry = 0;
		}
	}

	//Deinitialize solver
	if (object->solver) {
		if (object->solver->OnDeinitialize) object->solver->OnDeinitialize(object->system,object->solver,object);
	}

#ifndef EVDS_SINGLETHREADED
	//Free object from its confines (assuming that "Destroy" is matched with "Create")
	EVDS_Object_Release(object);

	//Mark object as destroyed (from this moment no function will accept this object)
	object->destroyed = 1;

	//Add object to list of objects to destroy
	SIMC_List_Append(object->system->deleted_objects,object);
#else
	//Delete object right away
	EVDS_InternalObject_DestroyData(object);
#endif

	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Check if object is destroyed.
///
/// @evds_mt Object will no longer be considered valid if it stops existing. The function
///  returns 1 if object was destroyed. A thread that is actively using this object must
///  release it and stop working with it as soon as it can detect it becoming invalid.
///
/// @evds_st Always returns 0 (object is valid/not destroyed).
///
/// This function must only be used when it is guranteed that objects will not be cleaned up
/// before it may be called, for example if object is marked as stored.
///
/// @note The object may typically exist for a couple seconds after it was destroyed, providing
///       enough time for basic operations to complete. If more strict control over the process
///       is required, see EVDS_Object_Store(), EVDS_Object_Release() and EVDS_System_CleanupObjects().
///
/// @param[in] object Object to check
/// @param[out] is_destroyed Pointer to where the requested information must be written
///
/// @returns Error code, is object valid
/// @retval EVDS_OK "object" pointer is not null
/// @retval EVDS_ERROR_BAD_PARAMETER "object" pointer is null
////////////////////////////////////////////////////////////////////////////////
int EVDS_Object_IsDestroyed(EVDS_OBJECT* object, int* is_destroyed) {
	if (!object) return EVDS_ERROR_BAD_PARAMETER;
#ifndef EVDS_SINGLETHREADED
	*is_destroyed = object->destroyed;
#else
	*is_destroyed = 0;
#endif
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Destroy objects internal data structure.
///
/// Destroys all variables inside the object and all of its list and lock objects. Does not
/// touch objects userdata. The variables tree is removed recursively.
////////////////////////////////////////////////////////////////////////////////
int EVDS_InternalObject_DestroyData(EVDS_OBJECT* object) {
	SIMC_LIST_ENTRY* entry;
	if (!object) return EVDS_ERROR_BAD_PARAMETER;
#ifndef EVDS_SINGLETHREADED
	if (!object->destroyed) return EVDS_ERROR_BAD_STATE;
#endif

	//Destroy variables
	entry = SIMC_List_GetFirst(object->variables);
	while (entry) {
		EVDS_VARIABLE* variable = (EVDS_VARIABLE*)SIMC_List_GetData(object->variables,entry);
		SIMC_List_Stop(object->variables,entry); //Stop iterating

		if (EVDS_InternalVariable_DestroyData(variable) == EVDS_OK) { //Destroy object and restart
			entry = SIMC_List_GetFirst(object->variables);
		} else {
			entry = 0;
		}
	}

	//Free resources
	SIMC_List_Destroy(object->variables);
	SIMC_List_Destroy(object->children);
	SIMC_List_Destroy(object->raw_children);
	SIMC_SRW_Destroy(object->state_lock);
	SIMC_SRW_Destroy(object->previous_state_lock);

	//Free object
	free(object);
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Create a new object.
///
/// After object is created, it must be initialized with an EVDS_Object_Initialize() call before 
/// it can be used for simulation. All variables and fixed internal structures must be 
/// created before object is initialized.
///
/// @note Before object is initialized, it will not show up in most object lists (list of objects by
///       type, list of children, will not be returned with object queries). Uninitialized objects
///       will only appear in raw lists (see EVDS_Object_GetAllChildren()).
///
/// After object has been initialized, it is no longer possible to alter the list of variables in it.
/// It will only be possible to  modify the existing variable values. 
///
/// Objects will be assigned an unique ID automatically, which can be changed later before the object is initialized
/// using EVDS_Object_SetUID().
/// Objects have empty name and type by default, but will be assigned a unique name if two objects with no name are
/// listed under the same parent.
///
/// Example of use:
/// ~~~{.c}
///     EVDS_OBJECT* earth;
///		EVDS_Object_Create(system,inertial_system,&earth);
///		EVDS_Object_SetType(earth,"planet");
///		EVDS_Object_SetName(earth,"Earth");
///		EVDS_Object_AddFloatVariable(earth,"mass",5.97e24,0);		//kg
///		EVDS_Object_AddFloatVariable(earth,"mu",3.9860044e14,0);	//m3 sec-2
///		EVDS_Object_AddFloatVariable(earth,"radius",6378.145e3,0);	//m
///		EVDS_Object_Initialize(earth,1);
/// ~~~
///
/// Children objects will be automatically initialized before parents initialization is complete.
/// See EVDS_Object_Initialize() for more information on initialization.
///
/// @param[in] system Pointer to EVDS_SYSTEM
/// @param[in] parent Parent object. If parent is not specified, the object created
///		as the child of the root object.
/// @param[out] p_object Pointer to the new EVDS_OBJECT structure will be written
///
/// @returns Error code
/// @retval EVDS_OK Successfully completed
/// @retval EVDS_ERROR_BAD_PARAMETER "system" is null
/// @retval EVDS_ERROR_BAD_PARAMETER "p_object" is null
/// @retval EVDS_ERROR_MEMORY Could not allocate memory for EVDS_OBJECT
////////////////////////////////////////////////////////////////////////////////
int EVDS_Object_Create(EVDS_SYSTEM* system, EVDS_OBJECT* parent, EVDS_OBJECT** p_object)
{
	EVDS_OBJECT* object;
	if (!system) return EVDS_ERROR_BAD_PARAMETER;
	if (!p_object) return EVDS_ERROR_BAD_PARAMETER;

	//Create new object
	object = (EVDS_OBJECT*)malloc(sizeof(EVDS_OBJECT));
	*p_object = object;
	if (!object) return EVDS_ERROR_MEMORY;
	memset(object,0,sizeof(EVDS_OBJECT));

	//If no parent defined, assume root object
	if (!parent) parent = system->inertial_space;

	//Object may be stored externally, the data it contains cannot be removed while it is still stored
	object->system = system;
	object->parent = parent;
	object->initialized = 0;
#ifndef EVDS_SINGLETHREADED
	object->initialize_thread = SIMC_THREAD_BAD_ID;
	object->integrate_thread = SIMC_THREAD_BAD_ID;
	object->render_thread = SIMC_THREAD_BAD_ID;
	object->stored_counter = 1; //Stored by default, in p_object
	object->destroyed = 0;
	object->create_thread = SIMC_Thread_GetUniqueID();
	object->state_lock = SIMC_SRW_Create();
	object->previous_state_lock = SIMC_SRW_Create();
#endif
	object->uid = 100000+(system->uid_counter++); //FIXME: could it be more arbitrary

	//Variables list
	SIMC_List_Create(&object->variables,0);
	SIMC_List_Create(&object->children,1);
	SIMC_List_Create(&object->raw_children,1);

	//Add to the list of objects, and add to parent
	object->object_entry = SIMC_List_Append(system->objects,object);
	object->parent_entry = 0;
	object->rparent_entry = 0;
	object->type_entry = 0;

	//Initialize state vector to zero in parent object coordinates
	if (parent) {
		object->parent_level = parent->parent_level+1;
		object->rparent_entry = SIMC_List_Append(parent->raw_children,object);
		EVDS_StateVector_Initialize(&object->previous_state,parent);
		EVDS_StateVector_Initialize(&object->state,parent);
	} else {
		object->parent_level = 0;
		EVDS_StateVector_Initialize(&object->previous_state,object);
		EVDS_StateVector_Initialize(&object->state,object);
	}
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Create new object as a copy of a different object.
///
/// Creates a new object using EVDS_Object_Create() and copies all internal data
/// from source object to the new object.
///
/// Source and parent objects may belong to different EVDS_SYSTEM objects. It is
/// possible to copy data between two different simulations this way. If parent is
/// defined, the new object is created in parent objects EVDS_SYSTEM.
///
/// @note The new copy of an object will not be initialized, but it may contain variables
///       created during the initialization of the other object.
///       See EVDS_Object_CopySingle() for more information.
///
/// @evds_mt If source object is being simulated by the other thread, it may be copied
///		in an inconsitent state. There is no blocking during copy operation.
///
/// @param[in] source Pointer to the source object
/// @param[in] parent Parent object (can be null)
/// @param[out] p_object Pointer to the new EVDS_OBJECT structure will be written here
///
/// @returns Error code
/// @retval EVDS_OK Successfully completed
/// @retval EVDS_ERROR_BAD_PARAMETER "source" is null
/// @retval EVDS_ERROR_MEMORY Could not allocate memory for EVDS_OBJECT
////////////////////////////////////////////////////////////////////////////////
int EVDS_Object_Copy(EVDS_OBJECT* source, EVDS_OBJECT* parent, EVDS_OBJECT** p_object) {
	EVDS_OBJECT* object;
	SIMC_LIST_ENTRY* entry;
	if (!source) return EVDS_ERROR_BAD_PARAMETER;

	//Copy the object itself
	EVDS_ERRCHECK(EVDS_Object_CopySingle(source,parent,&object));

	//Copy children
	entry = SIMC_List_GetFirst(source->raw_children);
	while (entry) {	
		EVDS_Object_Copy((EVDS_OBJECT*)SIMC_List_GetData(source->raw_children,entry),object,0);
		entry = SIMC_List_GetNext(source->raw_children,entry);
	}

	if (p_object) *p_object = object;
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Create a copy of source objects children under a new parent.
///
/// Copies all children (including non-initialized ones) from one parent into another parent
/// using EVDS_Object_Copy() for each child.
///
/// @note See EVDS_Object_Copy() and EVDS_Object_CopySingle() for more information.
///       The state of objects copied may be inconsistent if the objects are being simulated.
///
/// @param[in] source_parent Pointer to the source parent object
/// @param[in] parent Pointer to target parent object (can be null)
///
/// @returns Error code
/// @retval EVDS_OK Successfully completed
/// @retval EVDS_ERROR_BAD_PARAMETER "source_parent" is null
/// @retval EVDS_ERROR_MEMORY Could not allocate memory for EVDS_OBJECT
////////////////////////////////////////////////////////////////////////////////
int EVDS_Object_CopyChildren(EVDS_OBJECT* source_parent, EVDS_OBJECT* parent) {
	SIMC_LIST_ENTRY* entry;
	if (!source_parent) return EVDS_ERROR_BAD_PARAMETER;

	//Copy children
	entry = SIMC_List_GetFirst(source_parent->raw_children);
	while (entry) {	
		int error_code = EVDS_Object_Copy((EVDS_OBJECT*)SIMC_List_GetData(source_parent->raw_children,entry),parent,0);
		if (error_code) {
			SIMC_List_Stop(source_parent->raw_children, entry);
			return error_code;
		}
		entry = SIMC_List_GetNext(source_parent->raw_children,entry);
	}
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Create new object as a copy of a different object, but do not copy its children.
///
/// Creates a new object using EVDS_Object_Create() and copies all internal data (excluding children)
/// from source object to the new object.
///
/// Source and parent objects may belong to different EVDS_SYSTEM objects. It is
/// possible to copy data between two different simulations this way. If parent is
/// defined, the new object is created in parent objects EVDS_SYSTEM.
///
/// The new copy of an object will not be initialized, but it may contain variables
/// created during the initialization of the other object.
/// The userdata pointer will be retained in the copy (but not the solvers userdata).
///
/// @note The function does not copy data to which userdata points - only the pointer itself!
///
/// If any vector or quaternion variables are stored inside an object, they will be copied
/// over. The coordinate system reference will be updated only if vector points to the object
/// being copied or if vector points to parent. In any other case, the variables will be referencing
/// the original objects (for example if vector is specified in coordinate system of a child object).
///
/// @note The new copy of an object will not be initialized, but it may contain variables
///       created during the initialization of the other object.
///
/// @evds_mt If source object is being simulated by the other thread, it may be copied
///		in an inconsitent state. There is no blocking during copy operation.
///
/// @param[in] source Pointer to the source object
/// @param[in] parent Parent object (can be null)
/// @param[out] p_object Pointer to the new EVDS_OBJECT structure will be written here
///
/// @returns Error code
/// @retval EVDS_OK Successfully completed
/// @retval EVDS_ERROR_BAD_PARAMETER "source" is null
/// @retval EVDS_ERROR_MEMORY Could not allocate memory for EVDS_OBJECT
////////////////////////////////////////////////////////////////////////////////
int EVDS_Object_CopySingle(EVDS_OBJECT* source, EVDS_OBJECT* parent, EVDS_OBJECT** p_object) {
	EVDS_OBJECT* object;
	SIMC_LIST_ENTRY* entry;
	if (!source) return EVDS_ERROR_BAD_PARAMETER;

	if (parent) {
		EVDS_Object_Create(parent->system,parent,&object);
	} else {
		EVDS_Object_Create(source->system,parent,&object);
	}
	strncpy(object->name,source->name,256);
	strncpy(object->type,source->type,256);

	SIMC_SRW_EnterRead(source->state_lock); //Copy state under a lock
	EVDS_StateVector_Copy(&object->state,&source->state);
	SIMC_SRW_LeaveRead(source->state_lock);

	//Copy userdata pointer
	object->userdata = source->userdata;

	//Update state coordinate system
	object->state.position.coordinate_system = parent;
	object->state.velocity.coordinate_system = parent;
	object->state.acceleration.coordinate_system = parent;
	object->state.orientation.coordinate_system = parent;
	object->state.angular_velocity.coordinate_system = parent;
	object->state.angular_acceleration.coordinate_system = parent;

	//Copy variables
	entry = SIMC_List_GetFirst(source->variables);
	while (entry) {
		char name[65];
		EVDS_VARIABLE* source_value;
		EVDS_VARIABLE* value;

		source_value = (EVDS_VARIABLE*)SIMC_List_GetData(source->variables,entry);
		strncpy(name,source_value->name,64); name[64] = 0;

		EVDS_Object_AddVariable(object,name,source_value->type,&value);
		EVDS_Variable_Copy(source_value,value);

		//Try to correctly update coordinate systems of vectors and quaternions
		if (value->type == EVDS_VARIABLE_TYPE_VECTOR) {
			EVDS_VECTOR* vector = (EVDS_VECTOR*)value->value;
			if (vector-> coordinate_system == source->parent)	vector-> coordinate_system = parent;
			if (vector-> coordinate_system == source)			vector-> coordinate_system = object;
			if (vector->pcoordinate_system == source->parent)	vector->pcoordinate_system = parent;
			if (vector->pcoordinate_system == source)			vector->pcoordinate_system = object;
			if (vector->vcoordinate_system == source->parent)	vector->vcoordinate_system = parent;
			if (vector->vcoordinate_system == source)			vector->vcoordinate_system = object;
		} else if (value->type == EVDS_VARIABLE_TYPE_QUATERNION) {
			EVDS_QUATERNION* quaternion = (EVDS_QUATERNION*)value->value;
			if (quaternion->coordinate_system == source->parent)	quaternion->coordinate_system = parent;
			if (quaternion->coordinate_system == source)			quaternion->coordinate_system = object;
		}

		entry = SIMC_List_GetNext(source->variables,entry);
	}

	if (p_object) *p_object = object;
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Create a new object by origin object or return an already existing object.
///
/// This function will check if an object with given name (that depends on origin objects name and
/// target objects subname) already exists and return it. If it does not exist, a new object will
/// be created.
///
/// The name for the target object is defined as (including examples):
/// ~~~
///	origin_objects_name [sub_name]
/// Rocket engine [nozzle]
/// Gimbal [gimbal platform]
/// ~~~
///
/// The target use for this function is when solvers need to create some additional/new objects
/// during initialization. If EVDS_SYSTEM state is saved and then restored, the solvers must
/// find previously created object during initialization rather than creating a new one each time.
///
/// @note The created object may therefore be either empty or already have the variables defined.
///       If the object was created from scratch, EVDS_ERROR_NOT_FOUND error code is returned!
///
/// @param[in] origin Pointer to the origin object
/// @param[in] sub_name Sub-name of the target object
/// @param[in] parent Parent for the target object
/// @param[out] p_object Pointer to the new EVDS_OBJECT structure will be written here
///
/// @returns Error code
/// @retval EVDS_OK Successfully completed (object was already created before)
/// @retval EVDS_ERROR_NOT_FOUND Successfully completed (a new empty object was created)
/// @retval EVDS_ERROR_BAD_PARAMETER "origin" is null
/// @retval EVDS_ERROR_BAD_PARAMETER "sub_name" is null
/// @retval EVDS_ERROR_BAD_PARAMETER "p_object" is null
/// @retval EVDS_ERROR_MEMORY Could not allocate memory for EVDS_OBJECT
////////////////////////////////////////////////////////////////////////////////
int EVDS_Object_CreateBy(EVDS_OBJECT* origin, const char* sub_name, EVDS_OBJECT* parent, EVDS_OBJECT** p_object) {
	char full_name[257] = { 0 };
	if (!origin) return EVDS_ERROR_BAD_PARAMETER;
	if (!sub_name) return EVDS_ERROR_BAD_PARAMETER;
	if (!p_object) return EVDS_ERROR_BAD_PARAMETER;

	//Get full name of the sub-object
	snprintf(full_name,256,"%s [%s]",origin->name,sub_name);

	//Find this object inside parent or inside the entire system, or create new one
	if (EVDS_System_GetObjectByName(origin->system,full_name,parent,p_object) != EVDS_OK) {
		EVDS_ERRCHECK(EVDS_Object_Create(origin->system,parent,p_object));
		EVDS_ERRCHECK(EVDS_Object_SetName(*p_object,full_name));
		return EVDS_ERROR_NOT_FOUND;
	}
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Transfer initialization/creation right of object to the current thread.
///
/// This function will make it possible to initialize object (or finish defining its variables) in a different thread
/// (add/remove/edit variables list, etc). This will stop object from being accessible in the old thread.
///
/// The function must be called before EVDS_Object_Initialize() is called on this object.
/// If EVDS_Object_TransferInitialization() is called after EVDS_Object_Initialize(), but
/// before initialization is completed, the result is undefined.
///
/// @note Without transferring initialization ownership, all the data access functions from
///       other (non-initializing, non-creating) threads will fail.
///
/// @evds_st Does not do anything, only returns the error code
///
/// @param[in] object Object, initialization right for which must be transferred
///
/// @returns Error code
/// @retval EVDS_OK Successfully completed
/// @retval EVDS_ERROR_BAD_PARAMETER "object" is null
/// @retval EVDS_ERROR_BAD_STATE Object is already initialized
////////////////////////////////////////////////////////////////////////////////
int EVDS_Object_TransferInitialization(EVDS_OBJECT* object) {
	if (!object) return EVDS_ERROR_BAD_PARAMETER;
	if (object->initialized) return EVDS_ERROR_BAD_STATE;

#ifndef EVDS_SINGLETHREADED
	if (object->create_thread != SIMC_THREAD_BAD_ID) {
		object->create_thread = SIMC_Thread_GetUniqueID();
	}
	if (object->initialize_thread != SIMC_THREAD_BAD_ID) {
		object->initialize_thread = SIMC_Thread_GetUniqueID();
	}
#endif
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Check if object is initialized.
///
/// @evds_mt If object is still being initialized (or is not being initialized), will return 0
///
/// @evds_st Always returns 1
///
/// @param[in] object Object that must be checked
/// @param[out] is_initialized Returns 1 when object is already initialized, 0 if it's still initializing or not initialized yet
///
/// @returns Error code
/// @retval EVDS_OK Successfully completed
/// @retval EVDS_ERROR_BAD_PARAMETER "object" is null
/// @retval EVDS_ERROR_BAD_PARAMETER "is_initialized" is null
////////////////////////////////////////////////////////////////////////////////
int EVDS_Object_IsInitialized(EVDS_OBJECT* object, int* is_initialized) {
	if (!object) return EVDS_ERROR_BAD_PARAMETER;
	if (!is_initialized) return EVDS_ERROR_BAD_PARAMETER;
	*is_initialized = object->initialized;
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Signal that object is stored somewhere. Increments the reference counter.
///
/// Object is automatically stored when EVDS_Object_Create() is called. Object must
/// be stored before it is passed over to a different thread, but released
/// after that thread stops working with it.
///
/// The application may store object once, and never release it until it confirms
/// (using some internal means) that object is no longer used anywhere (for example,
/// deletes all data which may be relevant to this one).
///
/// @evds_st Increments reference counter by one. The object will be destroyed if reference
///  counter reaches zero (it defaults to 1 when object is created).
///
/// @param[in] object Object to be stored
///
/// @returns Error code
/// @retval EVDS_OK Successfully completed
/// @retval EVDS_ERROR_BAD_PARAMETER "object" is null
/// @retval EVDS_ERROR_BAD_STATE Object is destroyed
////////////////////////////////////////////////////////////////////////////////
int EVDS_Object_Store(EVDS_OBJECT* object) {
	if (!object) return EVDS_ERROR_BAD_PARAMETER;
#ifndef EVDS_SINGLETHREADED
	if (object->destroyed) return EVDS_ERROR_BAD_STATE;
#	ifdef _WIN32
	InterlockedIncrement(&object->stored_counter);
#	else
	__sync_fetch_and_add(&object->stored_counter,1);
#	endif
#else
	object->stored_counter++;
#endif
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Signal that objects is no longer stored. Decrements the reference counter.
///
/// Object is automatically released when EVDS_Object_Destroy() is called. Application
/// should call release in the thread that has finished work with the object.
///
/// The function will return an error if objects reference counter is already zero.
///
/// @evds_st Decrements the reference counter. If the counter reaches zero, the object
///  will be destroyed using EVDS_Object_Destroy().
///
/// @param[in] object Object to be released
///
/// @returns Error code
/// @retval EVDS_OK Successfully completed
/// @retval EVDS_ERROR_BAD_PARAMETER "object" is null
/// @retval EVDS_ERROR_INVALID_OBJECT Trying to release object which is no longer stored anywhere
////////////////////////////////////////////////////////////////////////////////
int EVDS_Object_Release(EVDS_OBJECT* object) {
	if (!object) return EVDS_ERROR_BAD_PARAMETER;
#ifndef EVDS_SINGLETHREADED
#	ifdef _WIN32
	if (InterlockedDecrement(&object->stored_counter) < 0) {
		InterlockedIncrement(&object->stored_counter);
		return EVDS_ERROR_INVALID_OBJECT;
    }
#	else
	if (__sync_fetch_and_add(&object->stored_counter,-1) < 0) {
        __sync_fetch_and_add(&object->stored_counter,1);
        return EVDS_ERROR_INVALID_OBJECT;
    }
#	endif
#else
	object->stored_counter--;
	if (object->stored_counter < 0) return EVDS_ERROR_INVALID_OBJECT;
	if (object->stored_counter == 0) EVDS_Object_Destroy(object);
#endif
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Set objects solver.
///
/// Set a custom callback used for solving the object. See EVDS_Object_Solve() for more information.
/// The custom callback will override solvers callback.
///
/// @returns Error code
/// @retval EVDS_OK Successfully completed
/// @retval EVDS_ERROR_BAD_PARAMETER "object" is null
/// @retval EVDS_ERROR_INVALID_OBJECT Object was destroyed
////////////////////////////////////////////////////////////////////////////////
int EVDS_Object_SetCallback_OnSolve(EVDS_OBJECT* object, EVDS_Callback_Solve* p_callback) {
	if (!object) return EVDS_ERROR_BAD_PARAMETER;
#ifndef EVDS_SINGLETHREADED
	if (object->destroyed) return EVDS_ERROR_INVALID_OBJECT;
#endif

	object->solve = p_callback;
	return EVDS_OK;
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Set objects integrator.
///
/// Set a custom callback used for integrating the objects state. See EVDS_Object_Integrate() for more information.
/// The custom callback will override solvers callback.
///
/// @returns Error code
/// @retval EVDS_OK Successfully completed
/// @retval EVDS_ERROR_BAD_PARAMETER "object" is null
/// @retval EVDS_ERROR_INVALID_OBJECT Object was destroyed
////////////////////////////////////////////////////////////////////////////////
int EVDS_Object_SetCallback_OnIntegrate(EVDS_OBJECT* object, EVDS_Callback_Integrate* p_callback) {
	if (!object) return EVDS_ERROR_BAD_PARAMETER;
#ifndef EVDS_SINGLETHREADED
	if (object->destroyed) return EVDS_ERROR_INVALID_OBJECT;
#endif

	object->integrate = p_callback;
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Sets object type. @evds_init_only
///
/// Object type determines which solver will be used for this object. Solvers typically
/// check for object type before they decide to claim the object.
///
/// Type must be a null-terminated C string, or a string of 256 characters (no null
/// termination is required then)
///
/// @param[in] object Pointer to object
/// @param[in] type Type (null-terminated string, only first 256 characters are taken)
///
/// @returns Error code
/// @retval EVDS_OK Successfully completed
/// @retval EVDS_ERROR_BAD_PARAMETER "object" is null
/// @retval EVDS_ERROR_BAD_PARAMETER "type" is null
/// @retval EVDS_ERROR_BAD_STATE Object was already initialized
/// @retval EVDS_ERROR_INVALID_OBJECT Object was destroyed
/// @retval EVDS_ERROR_INTERTHREAD_CALL The function can only be called from thread that is initializing the object 
///  (or thread that has created the object before initializer was called)
////////////////////////////////////////////////////////////////////////////////
int EVDS_Object_SetType(EVDS_OBJECT* object, const char* type) {
	if (!object) return EVDS_ERROR_BAD_PARAMETER;
	if (!type) return EVDS_ERROR_BAD_PARAMETER;
	if (object->initialized) return EVDS_ERROR_BAD_STATE;
#ifndef EVDS_SINGLETHREADED
	if (object->destroyed) return EVDS_ERROR_INVALID_OBJECT;
	if ((object->create_thread != SIMC_Thread_GetUniqueID()) &&
		(object->initialize_thread != SIMC_Thread_GetUniqueID())) return EVDS_ERROR_INTERTHREAD_CALL;
#endif

	//Set type in object
	strncpy(object->type,type,256);
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Sets object name. @evds_init_only
///
/// Object name usually has no special meaning other than to distinguish the object from
/// others. Some code (solvers or rendering code) may execute special behavior based on
/// the object name.
///
/// Name must be a null-terminated C string, or a string of 256 characters (no null
/// termination is required then)
///
/// The name must be unique between all children under the same parent. If the name is not unique,
/// the object will be automatically renamed after it's initialized.
///
/// The objects name must not contain the following special characters:
/// `*`, `/`, `[`, `]`.
///
/// @param[in] object Pointer to object
/// @param[in] name Name (null-terminated string, only first 256 characters are taken)
///
/// @returns Error code
/// @retval EVDS_OK Successfully completed
/// @retval EVDS_ERROR_BAD_PARAMETER "object" is null
/// @retval EVDS_ERROR_BAD_PARAMETER "name" is null
/// @retval EVDS_ERROR_BAD_STATE Object was already initialized
/// @retval EVDS_ERROR_INVALID_OBJECT Object was destroyed
/// @retval EVDS_ERROR_INTERTHREAD_CALL The function can only be called from thread that is initializing the object 
///  (or thread that has created the object before initializer was called)
////////////////////////////////////////////////////////////////////////////////
int EVDS_Object_SetName(EVDS_OBJECT* object, const char* name) {
	int count;
	char clean_name[256];
	char* clean_name_ptr;
	if (!object) return EVDS_ERROR_BAD_PARAMETER;
	if (!name) return EVDS_ERROR_BAD_PARAMETER;
	if (object->initialized) return EVDS_ERROR_BAD_STATE;
#ifndef EVDS_SINGLETHREADED
	if (object->destroyed) return EVDS_ERROR_INVALID_OBJECT;
	if ((object->create_thread != SIMC_Thread_GetUniqueID()) &&
		(object->initialize_thread != SIMC_Thread_GetUniqueID())) return EVDS_ERROR_INTERTHREAD_CALL;
#endif

	//Sanitize the name
	clean_name_ptr = clean_name;
	for (count = 1; (count <= 256) && (*name); 
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
	if (count < 256) *clean_name_ptr = '\0';

	//Store it
	strncpy(object->name,clean_name,256);
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Set objects name to a unique string.
///
/// This function call can be used to assign a globally unique name to the object.
///
/// @param[in] object Pointer to object
///
/// @returns Error code
/// @retval EVDS_OK Successfully completed
/// @retval EVDS_ERROR_BAD_PARAMETER "object" is null
/// @retval EVDS_ERROR_BAD_STATE Object was already initialized
/// @retval EVDS_ERROR_INVALID_OBJECT Object was destroyed
/// @retval EVDS_ERROR_INTERTHREAD_CALL The function can only be called from thread that is initializing the object 
///  (or thread that has created the object before initializer was called)
////////////////////////////////////////////////////////////////////////////////
int EVDS_Object_SetUniqueName(EVDS_OBJECT* object) {
	if (!object) return EVDS_ERROR_BAD_PARAMETER;
	if (object->initialized) return EVDS_ERROR_BAD_STATE;
#ifndef EVDS_SINGLETHREADED
	if (object->destroyed) return EVDS_ERROR_INVALID_OBJECT;
	if ((object->create_thread != SIMC_Thread_GetUniqueID()) &&
		(object->initialize_thread != SIMC_Thread_GetUniqueID())) return EVDS_ERROR_INTERTHREAD_CALL;
#endif

	snprintf(object->name,256,"@%4X%4X",rand(),rand()); //FIXME: better unique name
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Adds a variable. @evds_init_only
///
/// A new variable will be created inside the EVDS_OBJECT structure. The variable contents
/// will be cleared out. If variable is a vector or a quaternion, it must be additionally
/// initialized by user afterwards (using EVDS_Vector_Set() or a similar call).
///
/// For floating point variables, a shortcut call EVDS_Object_AddFloatVariable() is available.
///
/// Example of adding variables:
/// ~~~{.c}
///		EVDS_VARIABLE* variable;
///		EVDS_VECTOR vector;
///		EVDS_Object_AddVariable(object,"variable_name",EVDS_VARIABLE_TYPE_VECTOR,&variable);
///		EVDS_Vector_Set(&vector,object,EVDS_VECTOR_VELOCITY,0.0,0.0,0.0);
///		EVDS_Variable_SetVector(variable,&vector);
/// ~~~
///
/// See documentation on EVDS_VARIABLE for more information on variable types and API to use.
///
/// @evds_mt All normal variable types are thread safe. Custom data structures are not thread safe
/// and must be protected manually by the user.
///
/// @param[in] object Pointer to object
/// @param[in] name Variable name (null-terminated string, only first 64 characters are taken)
/// @param[in] type Variable type (see EVDS_VARIABLE)
/// @param[out] p_variable Variable pointer will be written here
///
/// @returns Error code
/// @retval EVDS_OK Successfully completed
/// @retval EVDS_ERROR_BAD_PARAMETER "object" is null
/// @retval EVDS_ERROR_BAD_PARAMETER "name" is null
/// @retval EVDS_ERROR_BAD_STATE Object was already initialized
/// @retval EVDS_ERROR_INVALID_OBJECT Object was destroyed
/// @retval EVDS_ERROR_INTERTHREAD_CALL The function can only be called from thread that is initializing the object 
///  (or thread that has created the object before initializer was called)
/// @retval EVDS_ERROR_MEMORY Error allocating EVDS_VARIABLE data structure
////////////////////////////////////////////////////////////////////////////////
int EVDS_Object_AddVariable(EVDS_OBJECT* object, const char* name, EVDS_VARIABLE_TYPE type, EVDS_VARIABLE** p_variable) {
	int error_code;
	EVDS_VARIABLE* variable;
	if (!object) return EVDS_ERROR_BAD_PARAMETER;
	if (!name) return EVDS_ERROR_BAD_PARAMETER;
	if (!p_variable) return EVDS_ERROR_BAD_PARAMETER;
	if (object->initialized) return EVDS_ERROR_BAD_STATE;
#ifndef EVDS_SINGLETHREADED
	if (object->destroyed) return EVDS_ERROR_INVALID_OBJECT;
	if ((object->create_thread != SIMC_Thread_GetUniqueID()) &&
		(object->initialize_thread != SIMC_Thread_GetUniqueID())) return EVDS_ERROR_INTERTHREAD_CALL;
#endif

	//Get variable
	error_code = EVDS_Object_GetVariable(object,name,&variable);
	if (error_code == EVDS_ERROR_NOT_FOUND) {
		error_code = EVDS_Variable_Create(object->system,name,type,&variable);
	}

	//Set parameters
	if ((error_code == EVDS_OK) && (!variable->object)) {
		variable->parent = 0;
		variable->object = object;
		variable->list_entry = SIMC_List_Append(object->variables,variable);
	}

	//Write back variable
	if (p_variable) *p_variable = variable;
	return error_code;
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Adds a floating point variable with default value. @evds_init_only
///
/// See EVDS_Object_AddVariable() for more documentation, and EVDS_VARIABLE for documentation on
/// using floating point variables and variables in general. Example of use:
/// ~~~{.c}
///		EVDS_VARIABLE* mu;
///		EVDS_Object_AddFloatVariable(earth,"radius",6378.145e3,0);
///		EVDS_Object_AddFloatVariable(earth,"mu",3.9860044e14,&mu);
/// ~~~
///
/// Unlike EVDS_Object_AddVariable(), this call will reset value of the floating point variable if it
/// is already defined! This call must not be used for variables which may be loaded from file.
///
/// This API call is typically used to set some numerical parameters and constants inside an object.
///
///
/// @param[in] object Pointer to object
/// @param[in] name Variable name (null-terminated string, only first 64 characters are taken)
/// @param[in] value Default value of the variable
/// @param[out] p_variable Variable pointer will be written here. If parameter is zero, it is ignored
///
/// @returns Error code
/// @retval EVDS_OK Successfully completed
/// @retval EVDS_ERROR_BAD_PARAMETER "object" is null
/// @retval EVDS_ERROR_BAD_PARAMETER "name" is null
/// @retval EVDS_ERROR_BAD_STATE Object was already initialized
/// @retval EVDS_ERROR_INVALID_OBJECT Object was destroyed
/// @retval EVDS_ERROR_INTERTHREAD_CALL The function can only be called from thread that is initializing the object 
///  (or thread that has created the object before initializer was called)
/// @retval EVDS_ERROR_MEMORY Error allocating EVDS_VARIABLE data structure
////////////////////////////////////////////////////////////////////////////////
int EVDS_Object_AddFloatVariable(EVDS_OBJECT* object, const char* name, EVDS_REAL value, EVDS_VARIABLE** p_variable) {
	int error_code;
	EVDS_VARIABLE* variable;
	if (!object) return EVDS_ERROR_BAD_PARAMETER;
	if (!name) return EVDS_ERROR_BAD_PARAMETER;
	if (object->initialized) return EVDS_ERROR_BAD_STATE;
#ifndef EVDS_SINGLETHREADED
	if (object->destroyed) return EVDS_ERROR_INVALID_OBJECT;
	if ((object->create_thread != SIMC_Thread_GetUniqueID()) &&
		(object->initialize_thread != SIMC_Thread_GetUniqueID())) return EVDS_ERROR_INTERTHREAD_CALL;
#endif

	//Get variable
	error_code = EVDS_Object_GetVariable(object,name,&variable);
	if (error_code == EVDS_ERROR_NOT_FOUND) {
		error_code = EVDS_Object_AddVariable(object,name,EVDS_VARIABLE_TYPE_FLOAT,&variable);
		if (error_code != EVDS_OK) return error_code;
		error_code = EVDS_Variable_SetReal(variable,value);
		if (error_code != EVDS_OK) return error_code;
	}

	if (p_variable) *p_variable = variable;
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Sets object unique identifier.
///
/// Despite being an unique identifier, careless user may cause two objects exist
/// under the same UID. Only one of the two objects will be returned by the API.
///
/// @param[in] object Pointer to object
/// @param[in] uid Any unsigned integer value
///
/// @returns Error code
/// @retval EVDS_OK Successfully completed
/// @retval EVDS_ERROR_BAD_PARAMETER "object" is null
/// @retval EVDS_ERROR_INVALID_OBJECT Object was destroyed
////////////////////////////////////////////////////////////////////////////////
int EVDS_Object_SetUID(EVDS_OBJECT* object, unsigned int uid) {
	if (!object) return EVDS_ERROR_BAD_PARAMETER;
#ifndef EVDS_SINGLETHREADED
	if (object->destroyed) return EVDS_ERROR_INVALID_OBJECT;
#endif

	object->uid = uid;
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Get objects unique identifier.
///
/// See EVDS_Object_SetUID()
///
/// @param[in] object Pointer to object
/// @param[out] uid Pointer to an unsigned integer
///
/// @returns Error code, unique identifier
/// @retval EVDS_OK Successfully completed
/// @retval EVDS_ERROR_BAD_PARAMETER "object" is null
/// @retval EVDS_ERROR_BAD_PARAMETER "uid" is null
/// @retval EVDS_ERROR_INVALID_OBJECT Object was destroyed
////////////////////////////////////////////////////////////////////////////////
int EVDS_Object_GetUID(EVDS_OBJECT* object, unsigned int* uid) {
	if (!object) return EVDS_ERROR_BAD_PARAMETER;
	if (!uid) return EVDS_ERROR_BAD_PARAMETER;
#ifndef EVDS_SINGLETHREADED
	if (object->destroyed) return EVDS_ERROR_INVALID_OBJECT;
#endif

	*uid = object->uid;
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Check object type. @evds_limited_init
///
/// Type must point to a null-terminated string, or a string of 256 characters
/// (no null termination required then).
///
/// The objects type can end with '*' as a wildcard. This can only be used as the
/// last character in the input string (all following characters will be ignored).
///
/// @param[in] object Pointer to object
/// @param[in] type Type (null-terminated string, only first 256 characters are compared)
///
/// @returns Error code
/// @retval EVDS_OK Successfully completed (object matches type)
/// @retval EVDS_ERROR_INVALID_TYPE Object is not of the given type
/// @retval EVDS_ERROR_BAD_PARAMETER "object" is null
/// @retval EVDS_ERROR_BAD_PARAMETER "type" is null
/// @retval EVDS_ERROR_INVALID_OBJECT Object was destroyed
/// @retval EVDS_ERROR_INTERTHREAD_CALL The function can only be called from thread that is initializing the object 
///  (or thread that has created the object before initializer was called)
////////////////////////////////////////////////////////////////////////////////
int EVDS_Object_CheckType(EVDS_OBJECT* object, const char* type) {
	int max_count;
	char* wildcard_pos;
	if (!object) return EVDS_ERROR_BAD_PARAMETER;
	if (!type) return EVDS_ERROR_BAD_PARAMETER;
#ifndef EVDS_SINGLETHREADED
	if (object->destroyed) return EVDS_ERROR_INVALID_OBJECT;
	if (!object->initialized &&
		(object->create_thread != SIMC_Thread_GetUniqueID()) &&
		(object->initialize_thread != SIMC_Thread_GetUniqueID())) return EVDS_ERROR_INTERTHREAD_CALL;
#endif

	//Determine maximum count to check
	max_count = 256;
	wildcard_pos = strchr(type,'*');
	if (wildcard_pos) max_count = wildcard_pos - type;
	if (max_count > 256) max_count = 256;

	//Check the object type
	if (strncmp(type,object->type,max_count) == 0) {
		return EVDS_OK;
	} else {
		return EVDS_ERROR_INVALID_TYPE;
	}
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Get object type. @evds_limited_init
///
/// Returns a string no more than max_length characters long. It may not be null
/// terminated. This is a correct way to get full type name:
/// ~~~{.c}
///		char type[257]; //256 plus null terminator
///		EVDS_Object_GetType(object,type,256);
///		type[256] = '\0'; //Null-terminate type
/// ~~~
///
/// @param[in] object Pointer to object
/// @param[out] type Pointer to type string
/// @param[in] max_length Maximum length of type string (no more than 256 needed)
///
/// @returns Error code, object type
/// @retval EVDS_OK Successfully completed
/// @retval EVDS_ERROR_NOT_FOUND Object is not of the given type
/// @retval EVDS_ERROR_BAD_PARAMETER "object" is null
/// @retval EVDS_ERROR_BAD_PARAMETER "type" is null
/// @retval EVDS_ERROR_INVALID_OBJECT Object was destroyed
/// @retval EVDS_ERROR_INTERTHREAD_CALL The function can only be called from thread that is initializing the object 
///  (or thread that has created the object before initializer was called)
////////////////////////////////////////////////////////////////////////////////
int EVDS_Object_GetType(EVDS_OBJECT* object, char* type, size_t max_length) {
	if (!object) return EVDS_ERROR_BAD_PARAMETER;
	if (!type) return EVDS_ERROR_BAD_PARAMETER;
#ifndef EVDS_SINGLETHREADED
	if (object->destroyed) return EVDS_ERROR_INVALID_OBJECT;
	if (!object->initialized &&
		(object->create_thread != SIMC_Thread_GetUniqueID()) &&
		(object->initialize_thread != SIMC_Thread_GetUniqueID())) return EVDS_ERROR_INTERTHREAD_CALL;
#endif

	strncpy(type,object->type,(max_length > 256 ? 256 : max_length));
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Get object name. @evds_limited_init
///
/// Returns a string no more than max_length characters long. It may not be null
/// terminated. This is a correct way to get full name:
/// ~~~{.c}
///		char name[257]; //256 plus null terminator
///		EVDS_Object_GetName(object,name,256);
///		name[256] = '\0'; //Null-terminate name
/// ~~~
///
/// @param[in] object Pointer to object
/// @param[out] name Pointer to name string
/// @param[in] max_length Maximum length of name string (no more than 256 needed)
///
/// @returns Error code, object name
/// @retval EVDS_OK Successfully completed
/// @retval EVDS_ERROR_BAD_PARAMETER "object" is null
/// @retval EVDS_ERROR_BAD_PARAMETER "name" is null
/// @retval EVDS_ERROR_INVALID_OBJECT Object was destroyed
/// @retval EVDS_ERROR_INTERTHREAD_CALL The function can only be called from thread that is initializing the object 
///  (or thread that has created the object before initializer was called)
////////////////////////////////////////////////////////////////////////////////
int EVDS_Object_GetName(EVDS_OBJECT* object, char* name, size_t max_length) {
	if (!object) return EVDS_ERROR_BAD_PARAMETER;
	if (!name) return EVDS_ERROR_BAD_PARAMETER;
#ifndef EVDS_SINGLETHREADED
	if (object->destroyed) return EVDS_ERROR_INVALID_OBJECT;
	if (!object->initialized &&
		(object->create_thread != SIMC_Thread_GetUniqueID()) &&
		(object->initialize_thread != SIMC_Thread_GetUniqueID())) return EVDS_ERROR_INTERTHREAD_CALL;
#endif

	strncpy(name,object->name,(max_length > 256 ? 256 : max_length));
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Get variable by name. @evds_limited_init
///
/// @param[in] object Pointer to object
/// @param[in] name Variable name (only first 256 characters are compared)
/// @param[out] p_variable Variable pointer is written here
///
/// @returns Error code, pointer to EVDS_VARIABLE
/// @retval EVDS_OK Successfully completed
/// @retval EVDS_ERROR_NOT_FOUND Variable not found in object
/// @retval EVDS_ERROR_BAD_PARAMETER "object" is null
/// @retval EVDS_ERROR_BAD_PARAMETER "name" is null
/// @retval EVDS_ERROR_BAD_PARAMETER "p_variable" is null
/// @retval EVDS_ERROR_INVALID_OBJECT Object was destroyed
/// @retval EVDS_ERROR_INTERTHREAD_CALL The function can only be called from thread that is initializing the object 
///  (or thread that has created the object before initializer was called)
////////////////////////////////////////////////////////////////////////////////
int EVDS_Object_GetVariable(EVDS_OBJECT* object, const char* name, EVDS_VARIABLE** p_variable) {
	SIMC_LIST_ENTRY* entry;
	if (!object) return EVDS_ERROR_BAD_PARAMETER;
	if (!name) return EVDS_ERROR_BAD_PARAMETER;
	if (!p_variable) return EVDS_ERROR_BAD_PARAMETER;
#ifndef EVDS_SINGLETHREADED
	if (object->destroyed) return EVDS_ERROR_INVALID_OBJECT;
	if (!object->initialized &&
		(object->create_thread != SIMC_Thread_GetUniqueID()) &&
		(object->initialize_thread != SIMC_Thread_GetUniqueID())) return EVDS_ERROR_INTERTHREAD_CALL;
#endif

	entry = SIMC_List_GetFirst(object->variables);
	while (entry) {
		EVDS_VARIABLE* variable = SIMC_List_GetData(object->variables,entry);
		if (strncmp(name,variable->name,64) == 0) {
			*p_variable = variable;
			SIMC_List_Stop(object->variables,entry);
			return EVDS_OK;
		}
		entry = SIMC_List_GetNext(object->variables,entry);
	}
	return EVDS_ERROR_NOT_FOUND;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Get floating-point variable by name. @evds_limited_init
///
/// This function will return null pointer for the variable and write 0.0 to value
/// if the variable does not exist.
///
/// @param[in] object Pointer to object
/// @param[in] name Variable name (only first 256 characters are compared)
/// @param[out] value Value of the variable will be written here
/// @param[out] p_variable Variable pointer is written here
///
/// @returns Error code, pointer to EVDS_VARIABLE
/// @retval EVDS_OK Successfully completed
/// @retval EVDS_ERROR_NOT_FOUND Variable not found in object
/// @retval EVDS_ERROR_BAD_PARAMETER "object" is null
/// @retval EVDS_ERROR_BAD_PARAMETER "name" is null
/// @retval EVDS_ERROR_INVALID_OBJECT Object was destroyed
/// @retval EVDS_ERROR_INTERTHREAD_CALL The function can only be called from thread that is initializing the object 
///  (or thread that has created the object before initializer was called)
////////////////////////////////////////////////////////////////////////////////
int EVDS_Object_GetRealVariable(EVDS_OBJECT* object, const char* name, EVDS_REAL* value, EVDS_VARIABLE** p_variable) {
	EVDS_VARIABLE* variable;
	if (!object) return EVDS_ERROR_BAD_PARAMETER;
	if (!name) return EVDS_ERROR_BAD_PARAMETER;
#ifndef EVDS_SINGLETHREADED
	if (object->destroyed) return EVDS_ERROR_INVALID_OBJECT;
	if (!object->initialized &&
		(object->create_thread != SIMC_Thread_GetUniqueID()) &&
		(object->initialize_thread != SIMC_Thread_GetUniqueID())) return EVDS_ERROR_INTERTHREAD_CALL;
#endif

	if (EVDS_Object_GetVariable(object,name,&variable) == EVDS_OK) {
		if (value) EVDS_Variable_GetReal(variable,value);
		if (p_variable) *p_variable = variable;
	} else {
		if (value) *value = 0.0;
		if (p_variable) *p_variable = 0;
	}
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Get list of all variables. @evds_limited_init
///
/// See SIMC_LIST documentation for an example on iterating through list. The
/// list will contain pointers to EVDS_VARIABLE structures:
/// ~~~{.c}
///		EVDS_VARIABLE* variable = (EVDS_VARIABLE*)SIMC_List_GetData(list,entry);
/// ~~~
///
/// @param[in] object Pointer to object
/// @param[out] p_list Pointer to list of all variables will be written here
///
/// @returns Error code, pointer to SIMC_LIST of variables
/// @retval EVDS_OK Successfully completed
/// @retval EVDS_ERROR_BAD_PARAMETER "object" is null
/// @retval EVDS_ERROR_BAD_PARAMETER "p_list" is null
/// @retval EVDS_ERROR_INVALID_OBJECT Object was destroyed
/// @retval EVDS_ERROR_INTERTHREAD_CALL The function can only be called from thread that is initializing the object 
///  (or thread that has created the object before initializer was called)
////////////////////////////////////////////////////////////////////////////////
int EVDS_Object_GetVariables(EVDS_OBJECT* object, SIMC_LIST** p_list) {
	if (!object) return EVDS_ERROR_BAD_PARAMETER;
	if (!p_list) return EVDS_ERROR_BAD_PARAMETER;
#ifndef EVDS_SINGLETHREADED
	if (object->destroyed) return EVDS_ERROR_INVALID_OBJECT;
	if (!object->initialized &&
		(object->create_thread != SIMC_Thread_GetUniqueID()) &&
		(object->initialize_thread != SIMC_Thread_GetUniqueID())) return EVDS_ERROR_INTERTHREAD_CALL;
#endif

	*p_list = object->variables;
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Get list of children that can be solved/worked with.
///
/// See SIMC_LIST documentation for an example on iterating through list. The
/// list will contain pointers to EVDS_OBJECT structures:
/// ~~~{.c}
///		EVDS_OBJECT* child = (EVDS_OBJECT*)SIMC_List_GetData(list,entry);
/// ~~~
///
/// @param[in] object Pointer to object
/// @param[out] p_list Pointer to list of initialized children will be written here
///
/// @returns Error code, pointer to SIMC_LIST of children
/// @retval EVDS_OK Successfully completed
/// @retval EVDS_ERROR_BAD_PARAMETER "object" is null
/// @retval EVDS_ERROR_BAD_PARAMETER "p_list" is null
/// @retval EVDS_ERROR_INVALID_OBJECT Object was destroyed
////////////////////////////////////////////////////////////////////////////////
int EVDS_Object_GetChildren(EVDS_OBJECT* object, SIMC_LIST** p_list) {
	if (!object) return EVDS_ERROR_BAD_PARAMETER;
	if (!p_list) return EVDS_ERROR_BAD_PARAMETER;
#ifndef EVDS_SINGLETHREADED
	if (object->destroyed) return EVDS_ERROR_INVALID_OBJECT;
#endif

	*p_list = object->children;
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Get list of all children (including children which were not initialized).
///
/// See SIMC_LIST documentation for an example on iterating through list. The
/// list will contain pointers to EVDS_OBJECT structures:
/// ~~~{.c}
///		EVDS_OBJECT* child = (EVDS_OBJECT*)SIMC_List_GetData(list,entry);
/// ~~~
///
/// @param[in] object Pointer to object
/// @param[out] p_list Pointer to list of all children will be written here
///
/// @returns Error code, pointer to SIMC_LIST of children
/// @retval EVDS_OK Successfully completed
/// @retval EVDS_ERROR_BAD_PARAMETER "object" is null
/// @retval EVDS_ERROR_BAD_PARAMETER "p_list" is null
/// @retval EVDS_ERROR_INVALID_OBJECT Object was destroyed
////////////////////////////////////////////////////////////////////////////////
int EVDS_Object_GetAllChildren(EVDS_OBJECT* object, SIMC_LIST** p_list) {
	if (!object) return EVDS_ERROR_BAD_PARAMETER;
	if (!p_list) return EVDS_ERROR_BAD_PARAMETER;
#ifndef EVDS_SINGLETHREADED
	if (object->destroyed) return EVDS_ERROR_INVALID_OBJECT;
#endif

	*p_list = object->raw_children;
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Get objects immediate parent object.
///
/// Some objects may have a changing parent object (for example vessels may be switched
/// between different inertial coordinate systems - this will cause parent object to change
/// with time).
///
/// This call is not completely thread safe as the parent changes after this API call.
///
/// @param[in] object Pointer to object
/// @param[out] p_object Pointer to parent object will be written here (a null pointer will be written for the root object)
///
/// @returns Error code, pointer to object
/// @retval EVDS_OK Successfully completed
/// @retval EVDS_ERROR_BAD_PARAMETER "object" is null
/// @retval EVDS_ERROR_BAD_PARAMETER "p_object" is null
/// @retval EVDS_ERROR_INVALID_OBJECT Object was destroyed
////////////////////////////////////////////////////////////////////////////////
int EVDS_Object_GetParent(EVDS_OBJECT* object, EVDS_OBJECT** p_object) {
	if (!object) return EVDS_ERROR_BAD_PARAMETER;
	if (!p_object) return EVDS_ERROR_BAD_PARAMETER;
#ifndef EVDS_SINGLETHREADED
	if (object->destroyed) return EVDS_ERROR_INVALID_OBJECT;
#endif

	*p_object = object->parent;
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Get the objects parent with the given type (can contain wildcards).
///
/// The returned pointer will be the first object which matches the given type,
/// to which the passed object or any of the passed objects parents belong.
///
/// This call is not completely thread safe as the parent changes after this API call.
///
/// @param[in] object Pointer to object
/// @param[out] p_object Pointer to coordinate system will be written here
///
/// @returns Error code, pointer to coordinate system
/// @retval EVDS_OK Successfully completed
/// @retval EVDS_ERROR_BAD_PARAMETER "object" is null
/// @retval EVDS_ERROR_BAD_PARAMETER "type" is null
/// @retval EVDS_ERROR_BAD_PARAMETER "p_object" is null
/// @retval EVDS_ERROR_INVALID_TYPE Parent not found, or is not of the given type
/// @retval EVDS_ERROR_INVALID_OBJECT Object was destroyed
/// @retval EVDS_ERROR_INTERTHREAD_CALL The function can only be called from thread that is initializing the object 
///  (or thread that has created the object before initializer was called)

////////////////////////////////////////////////////////////////////////////////
int EVDS_Object_GetParentObjectByType(EVDS_OBJECT* object, const char* type, EVDS_OBJECT** p_object) {
	EVDS_OBJECT* parent;
	if (!object) return EVDS_ERROR_BAD_PARAMETER;
	if (!type) return EVDS_ERROR_BAD_PARAMETER;
	if (!p_object) return EVDS_ERROR_BAD_PARAMETER;
#ifndef EVDS_SINGLETHREADED
	if (object->destroyed) return EVDS_ERROR_INVALID_OBJECT;
#endif

	parent = object;
	while (parent && (EVDS_Object_CheckType(parent,type) != EVDS_OK)) {
		parent = parent->parent;
	}
	*p_object = parent;
	if (!parent) {
		return EVDS_ERROR_INVALID_TYPE;
	} else {
		return EVDS_Object_CheckType(parent,type);
	}
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Change the objects parent.
///
/// @bug Not fully implemented, not fully documented
////////////////////////////////////////////////////////////////////////////////
int EVDS_Object_SetParent(EVDS_OBJECT* object, EVDS_OBJECT* new_parent) {
	//FIXME: lock object lists in parent and new parent
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Move object in front of the "head" object in list of children.
///
/// This function will only move children in the "raw" list (all children including
/// uninitialized ones). It will not affect the list which is actually used by
/// propagators/solvers when iterating.
///
/// In order to swap order in which children are iterated during simulation
/// all children must first be deinitialized and then reinitialized again with
/// new order. See EVDS_Object_Deinitialize() and EVDS_Object_Initialize().
///
/// @param[in] object Pointer to object which must be moved
/// @param[in] head Pointer to object which will be in front in list (can be null)
///
/// @returns Error code
/// @retval EVDS_OK Successfully completed
/// @retval EVDS_ERROR_BAD_PARAMETER "object" is null
/// @retval EVDS_ERROR_BAD_PARAMETER "head" does not have same parent as "object"
/// @retval EVDS_ERROR_BAD_PARAMETER object has no parent
/// @retval EVDS_ERROR_BAD_STATE Could not find "head" in list (object may have been destroyed)
/// @retval EVDS_ERROR_INVALID_OBJECT Object was destroyed
////////////////////////////////////////////////////////////////////////////////
int EVDS_Object_MoveInList(EVDS_OBJECT* object, EVDS_OBJECT* head) {
	SIMC_LIST_ENTRY* head_entry = 0;
	if (!object) return EVDS_ERROR_BAD_PARAMETER;
	if (!object->parent) return EVDS_ERROR_BAD_PARAMETER;
	if (head && (head->parent != object->parent)) return EVDS_ERROR_BAD_PARAMETER;
#ifndef EVDS_SINGLETHREADED
	if (object->destroyed) return EVDS_ERROR_INVALID_OBJECT;
#endif

	if (head) head_entry = head->rparent_entry;
	SIMC_List_GetFirst(object->parent->raw_children);
	SIMC_List_MoveInFront(object->parent->raw_children,object->rparent_entry,head_entry);
	return EVDS_OK;
}



////////////////////////////////////////////////////////////////////////////////
/// @brief Get a copy of the most recent objects state vector.
///
/// Example of use:
/// ~~~{.c}
///		EVDS_STATE_VECTOR state;
///		EVDS_Object_GetStateVector(object,&state);
/// ~~~
///
/// @param[in] object Pointer to object
/// @param[out] vector State vector will be copied by this pointer
///
/// @returns Error code, a copy of state vector
/// @retval EVDS_OK Successfully completed
/// @retval EVDS_ERROR_BAD_PARAMETER "object" is null
/// @retval EVDS_ERROR_BAD_PARAMETER "vector" is null
/// @retval EVDS_ERROR_INVALID_OBJECT Object was destroyed
////////////////////////////////////////////////////////////////////////////////
int EVDS_Object_GetStateVector(EVDS_OBJECT* object, EVDS_STATE_VECTOR* vector) {
	if (!object) return EVDS_ERROR_BAD_PARAMETER;
	if (!vector) return EVDS_ERROR_BAD_PARAMETER;
#ifndef EVDS_SINGLETHREADED
	if (object->destroyed) return EVDS_ERROR_INVALID_OBJECT;
#endif

	//Get state vector data
	SIMC_SRW_EnterRead(object->state_lock);
	memcpy(vector,&object->state,sizeof(EVDS_STATE_VECTOR));
	SIMC_SRW_LeaveRead(object->state_lock);
	
	//Update internal vector information (FIXME: revise this)
	EVDS_Vector_SetPositionVector(&vector->velocity,&vector->position);
	EVDS_Vector_SetPositionVector(&vector->acceleration,&vector->position);
	EVDS_Vector_SetPositionVector(&vector->angular_velocity,&vector->position);
	EVDS_Vector_SetPositionVector(&vector->angular_acceleration,&vector->position);

	EVDS_Vector_SetVelocityVector(&vector->acceleration,&vector->velocity);
	EVDS_Vector_SetVelocityVector(&vector->angular_velocity,&vector->velocity);
	EVDS_Vector_SetVelocityVector(&vector->angular_acceleration,&vector->velocity);
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Set state vector of an object.
///
/// State vector components will be converted to parent objects coordinate system.
///
/// Example of use:
/// ~~~{.c}
///		EVDS_STATE_VECTOR state;
///		EVDS_Vector_Set(&state.position,inertial_system,EVDS_VECTOR_POSITION,0.0,100.0,0.0);
///		EVDS_Object_SetStateVector(object,&state);
/// ~~~
///
/// @param[in] object Pointer to object
/// @param[in] vector State vector to use
///
/// @returns Error code
/// @retval EVDS_OK Successfully completed
/// @retval EVDS_ERROR_BAD_PARAMETER "object" is null
/// @retval EVDS_ERROR_BAD_PARAMETER "vector" is null
/// @retval EVDS_ERROR_INVALID_OBJECT Object was destroyed
////////////////////////////////////////////////////////////////////////////////
int EVDS_Object_SetStateVector(EVDS_OBJECT* object, EVDS_STATE_VECTOR* vector) {
	if (!object) return EVDS_ERROR_BAD_PARAMETER;
	if (!vector) return EVDS_ERROR_BAD_PARAMETER;
#ifndef EVDS_SINGLETHREADED
	if (object->destroyed) return EVDS_ERROR_INVALID_OBJECT;
#endif

	SIMC_SRW_EnterWrite(object->state_lock);
	SIMC_SRW_EnterRead(object->previous_state_lock);
	memcpy(&object->previous_state,&object->state,sizeof(EVDS_STATE_VECTOR));
	SIMC_SRW_LeaveRead(object->previous_state_lock);
	SIMC_SRW_LeaveWrite(object->state_lock);

	SIMC_SRW_EnterWrite(object->state_lock);
	memcpy(&object->state,vector,sizeof(EVDS_STATE_VECTOR));
	SIMC_SRW_LeaveWrite(object->state_lock);
#ifndef EVDS_SINGLETHREADED
	memcpy(&object->private_state,vector,sizeof(EVDS_STATE_VECTOR));
#endif
	return EVDS_OK;
}


#ifndef EVDS_SINGLETHREADED
////////////////////////////////////////////////////////////////////////////////
/// @brief Set private state vector derivative of an object.
///
/// Private state vector is used by EVDS_Object_Integrate(), and only means something
/// inside an EVDS_Object_Integrate() call.
////////////////////////////////////////////////////////////////////////////////
int EVDS_InternalObject_SetPrivateStateVector(EVDS_OBJECT* object, EVDS_STATE_VECTOR* vector) {
	if (!object) return EVDS_ERROR_BAD_PARAMETER;
	if (!vector) return EVDS_ERROR_BAD_PARAMETER;
#ifndef EVDS_SINGLETHREADED
	if (object->destroyed) return EVDS_ERROR_INVALID_OBJECT;
#endif

	memcpy(&object->private_state,vector,sizeof(EVDS_STATE_VECTOR));
	return EVDS_OK;
}
#endif


////////////////////////////////////////////////////////////////////////////////
/// @brief Set userdata pointer.
///
/// Userdata can be a pointer to any arbitrary data.
///
/// @param[in] object Pointer to object
/// @param[in] userdata Pointer to userdata
///
/// @returns Error code
/// @retval EVDS_OK Successfully completed
/// @retval EVDS_ERROR_BAD_PARAMETER "object" is null
/// @retval EVDS_ERROR_INVALID_OBJECT Object was destroyed
////////////////////////////////////////////////////////////////////////////////
int EVDS_Object_SetUserdata(EVDS_OBJECT* object, void* userdata) {
	if (!object) return EVDS_ERROR_BAD_PARAMETER;
#ifndef EVDS_SINGLETHREADED
	if (object->destroyed) return EVDS_ERROR_INVALID_OBJECT;
#endif
	object->userdata = userdata;
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Get userdata pointer.
///
/// Userdata can be a pointer to any arbitrary data.
///
/// @param[in] object Pointer to object
/// @param[out] p_userdata Pointer to userdata will be written here
///
/// @returns Error code, pointer to userdata
/// @retval EVDS_OK Successfully completed 
/// @retval EVDS_ERROR_BAD_PARAMETER "object" is null
/// @retval EVDS_ERROR_BAD_PARAMETER "userdata" is null
/// @retval EVDS_ERROR_INVALID_OBJECT Object was destroyed
////////////////////////////////////////////////////////////////////////////////
int EVDS_Object_GetUserdata(EVDS_OBJECT* object, void** p_userdata) {
	if (!object) return EVDS_ERROR_BAD_PARAMETER;
	if (!p_userdata) return EVDS_ERROR_BAD_PARAMETER;
#ifndef EVDS_SINGLETHREADED
	if (object->destroyed) return EVDS_ERROR_INVALID_OBJECT;
#endif
	*p_userdata = object->userdata;
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Set pointer to data, specific to objects solver.
///
/// This call is used internally by solvers to store additional data that's related
/// to the object.
///
/// @param[in] object Pointer to object
/// @param[in] solverdata Pointer to solverdata
///
/// @returns Error code
/// @retval EVDS_OK Successfully completed
/// @retval EVDS_ERROR_BAD_PARAMETER "object" is null
/// @retval EVDS_ERROR_INVALID_OBJECT Object was destroyed
////////////////////////////////////////////////////////////////////////////////
int EVDS_Object_SetSolverdata(EVDS_OBJECT* object, void* solverdata) {
	if (!object) return EVDS_ERROR_BAD_PARAMETER;
#ifndef EVDS_SINGLETHREADED
	if (object->destroyed) return EVDS_ERROR_INVALID_OBJECT;
#endif
	object->solverdata = solverdata;
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Get pointer to data, specific to objects solver.
///
/// This call is used internally by solvers to read additional data that's related
/// to the object.
///
/// @param[in] object Pointer to object
/// @param[out] solverdata Pointer to solverdata will be written here
///
/// @returns Error code, pointer to userdata
/// @retval EVDS_OK Successfully completed 
/// @retval EVDS_ERROR_BAD_PARAMETER "object" is null
/// @retval EVDS_ERROR_BAD_PARAMETER "solverdata" is null
/// @retval EVDS_ERROR_INVALID_OBJECT Object was destroyed
////////////////////////////////////////////////////////////////////////////////
int EVDS_Object_GetSolverdata(EVDS_OBJECT* object, void** solverdata) {
	if (!object) return EVDS_ERROR_BAD_PARAMETER;
	if (!solverdata) return EVDS_ERROR_BAD_PARAMETER;
#ifndef EVDS_SINGLETHREADED
	if (object->destroyed) return EVDS_ERROR_INVALID_OBJECT;
#endif
	*solverdata = object->solverdata;
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Set objects position in the target coordinates.
///
/// If target coordinates are null, objects parent will be used. If object is the root object, this call will fail.
///
/// @param[in] object Pointer to object
/// @param[in] target_coordinates Target coordinates in which x, y, z components are specified
/// @param[in] x X component of position
/// @param[in] y Y component of position
/// @param[in] z Z component of position
///
/// @returns Error code, pointer to userdata
/// @retval EVDS_OK Successfully completed (object matches type)
/// @retval EVDS_ERROR_BAD_PARAMETER "object" is null
/// @retval EVDS_ERROR_BAD_PARAMETER "target_coordinates" is null and object is the root object
/// @retval EVDS_ERROR_INVALID_OBJECT Object was destroyed
////////////////////////////////////////////////////////////////////////////////
int EVDS_Object_SetPosition(EVDS_OBJECT* object, EVDS_OBJECT* target_coordinates, EVDS_REAL x, EVDS_REAL y, EVDS_REAL z) {
	if (!object) return EVDS_ERROR_BAD_PARAMETER;
	if (!target_coordinates) target_coordinates = object->parent;
	if (!target_coordinates) return EVDS_ERROR_BAD_PARAMETER;
#ifndef EVDS_SINGLETHREADED
	if (object->destroyed) return EVDS_ERROR_INVALID_OBJECT;
#endif

	SIMC_SRW_EnterWrite(object->state_lock);
	EVDS_Vector_Set(&object->state.position,EVDS_VECTOR_POSITION,target_coordinates,x,y,z);
	SIMC_SRW_LeaveWrite(object->state_lock);
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Set objects velocity in the target coordinates.
///
/// If target coordinates are null, objects parent will be used. If object is the root object, this call will fail.
///
/// @param[in] object Pointer to object
/// @param[in] target_coordinates Target coordinates in which vx, vy, vz components are specified
/// @param[in] vx X component of velocity
/// @param[in] vy Y component of velocity
/// @param[in] vz Z component of velocity
///
/// @returns Error code, pointer to userdata
/// @retval EVDS_OK Successfully completed (object matches type)
/// @retval EVDS_ERROR_BAD_PARAMETER "object" is null
/// @retval EVDS_ERROR_BAD_PARAMETER "target_coordinates" is null and object is the root object
/// @retval EVDS_ERROR_INVALID_OBJECT Object was destroyed
////////////////////////////////////////////////////////////////////////////////
int EVDS_Object_SetVelocity(EVDS_OBJECT* object, EVDS_OBJECT* target_coordinates, EVDS_REAL vx, EVDS_REAL vy, EVDS_REAL vz) {
	if (!object) return EVDS_ERROR_BAD_PARAMETER;
	if (!target_coordinates) target_coordinates = object->parent;
	if (!target_coordinates) return EVDS_ERROR_BAD_PARAMETER;
#ifndef EVDS_SINGLETHREADED
	if (object->destroyed) return EVDS_ERROR_INVALID_OBJECT;
#endif

	SIMC_SRW_EnterWrite(object->state_lock);
	EVDS_Vector_Set(&object->state.velocity,EVDS_VECTOR_VELOCITY,target_coordinates,vx,vy,vz);
	SIMC_SRW_LeaveWrite(object->state_lock);
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Set objects orientation in the target coordinates.
///
/// If target coordinates are null, objects parent will be used. If object is the root object, this call will fail.
/// Use EVDS_RAD() macro to convert degrees to radians:
/// ~~~{.c}
///		EVDS_Object_SetOrientation(object,0,EVDS_RAD(45.0),EVDS_RAD(90.0),EVDS_RAD(0.0));
/// ~~~
///
/// @param[in] object Pointer to object
/// @param[in] target_coordinates Target coordinates in which vx, vy, vz components are specified
/// @param[in] roll X component of orientation (roll) in radians
/// @param[in] pitch Y component of orientation (pitch) in radians
/// @param[in] yaw Z component of orientation (yaw) in radians
///
/// @returns Error code, pointer to userdata
/// @retval EVDS_OK Successfully completed (object matches type)
/// @retval EVDS_ERROR_BAD_PARAMETER "object" is null
/// @retval EVDS_ERROR_BAD_PARAMETER "target_coordinates" is null and object is the root object
/// @retval EVDS_ERROR_INVALID_OBJECT Object was destroyed
////////////////////////////////////////////////////////////////////////////////
int EVDS_Object_SetOrientation(EVDS_OBJECT* object, EVDS_OBJECT* target_coordinates, EVDS_REAL roll, EVDS_REAL pitch, EVDS_REAL yaw) {
	if (!object) return EVDS_ERROR_BAD_PARAMETER;
	if (!target_coordinates) target_coordinates = object->parent;
	if (!target_coordinates) return EVDS_ERROR_BAD_PARAMETER;
#ifndef EVDS_SINGLETHREADED
	if (object->destroyed) return EVDS_ERROR_INVALID_OBJECT;
#endif

	SIMC_SRW_EnterWrite(object->state_lock);
	EVDS_Quaternion_SetEuler(&object->state.orientation,target_coordinates,roll,pitch,yaw);
	SIMC_SRW_LeaveWrite(object->state_lock);
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Set objects angular velocity in the target coordinates.
///
/// If target coordinates are null, objects parent will be used. If object is the root object, this call will fail.
///
/// @param[in] object Pointer to object
/// @param[in] target_coordinates Target coordinates in which vx, vy, vz components are specified
/// @param[in] r X component of angular velocity (roll rate) in radians per second
/// @param[in] p Y component of angular velocity (pitch rate) in radians per second
/// @param[in] q Z component of angular velocity (yaw rate) in radians per second
///
/// @returns Error code, pointer to userdata
/// @retval EVDS_OK Successfully completed (object matches type)
/// @retval EVDS_ERROR_BAD_PARAMETER "object" is null
/// @retval EVDS_ERROR_BAD_PARAMETER "target_coordinates" is null and object is the root object
/// @retval EVDS_ERROR_INVALID_OBJECT Object was destroyed
////////////////////////////////////////////////////////////////////////////////
int EVDS_Object_SetAngularVelocity(EVDS_OBJECT* object, EVDS_OBJECT* target_coordinates, EVDS_REAL r, EVDS_REAL p, EVDS_REAL q) {
	if (!object) return EVDS_ERROR_BAD_PARAMETER;
	if (!target_coordinates) target_coordinates = object->parent;
	if (!target_coordinates) return EVDS_ERROR_BAD_PARAMETER;
#ifndef EVDS_SINGLETHREADED
	if (object->destroyed) return EVDS_ERROR_INVALID_OBJECT;
#endif

	SIMC_SRW_EnterWrite(object->state_lock);
	EVDS_Vector_Set(&object->state.angular_velocity,EVDS_VECTOR_ANGULAR_VELOCITY,target_coordinates,r,p,q);
	SIMC_SRW_LeaveWrite(object->state_lock);
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Set time at which objects state vector is specified.
///
/// @param[in] object Pointer to object
/// @param[in] mjd_time Mean julian date
///
/// @returns Error code, pointer to userdata
/// @retval EVDS_OK Successfully completed (object matches type)
/// @retval EVDS_ERROR_BAD_PARAMETER "object" is null
/// @retval EVDS_ERROR_INVALID_OBJECT Object was destroyed
////////////////////////////////////////////////////////////////////////////////
int EVDS_Object_SetStateTime(EVDS_OBJECT* object, double mjd_time) {
	if (!object) return EVDS_ERROR_BAD_PARAMETER;
#ifndef EVDS_SINGLETHREADED
	if (object->destroyed) return EVDS_ERROR_INVALID_OBJECT;
#endif

	SIMC_SRW_EnterWrite(object->state_lock);
	object->state.time = mjd_time;
	SIMC_SRW_LeaveWrite(object->state_lock);
	return EVDS_OK;
}




////////////////////////////////////////////////////////////////////////////////
/// @brief Get a copy of the previous objects state vector.
///
/// This is the state vector that object had before last EVDS_Object_SetState()
/// API call. Useful for interpolation, but may not work if propagator or user code
/// makes multiple updates of state vector.
///
/// Example of use:
/// ~~~{.c}
///		EVDS_STATE_VECTOR state;
///		EVDS_Object_GetPreviousStateVector(object,&state);
/// ~~~
///
/// @param[in] object Pointer to object
/// @param[out] vector State vector will be copied by this pointer
///
/// @returns Error code, a copy of state vector
/// @retval EVDS_OK Successfully completed (object matches type)
/// @retval EVDS_ERROR_BAD_PARAMETER "object" is null
/// @retval EVDS_ERROR_BAD_PARAMETER "vector" is null
/// @retval EVDS_ERROR_INVALID_OBJECT Object was destroyed
////////////////////////////////////////////////////////////////////////////////
int EVDS_Object_GetPreviousStateVector(EVDS_OBJECT* object, EVDS_STATE_VECTOR* vector) {
	if (!object) return EVDS_ERROR_BAD_PARAMETER;
	if (!vector) return EVDS_ERROR_BAD_PARAMETER;
#ifndef EVDS_SINGLETHREADED
	if (object->destroyed) return EVDS_ERROR_INVALID_OBJECT;
#endif

	SIMC_SRW_EnterRead(object->previous_state_lock);
	memcpy(vector,&object->previous_state,sizeof(EVDS_STATE_VECTOR));
	SIMC_SRW_LeaveRead(object->previous_state_lock);
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Get a state vector linearly interpolated between current and previous state vectors.
///
/// This can be used for computing object positions linearly interpolated between two points.
///
/// Example of use:
/// ~~~{.c}
///		EVDS_STATE_VECTOR state;
///		EVDS_Object_GetInterpolatedStateVector(object,&state,0.5);
/// ~~~
///
/// @param[in] object Pointer to object
/// @param[out] vector State vector will be copied by this pointer
/// @param[in] t Interpolation time, \f$t \in [0.0 ... 1.0]\f$
///
/// @returns Error code, a copy of state vector
/// @retval EVDS_OK Successfully completed (object matches type)
/// @retval EVDS_ERROR_BAD_PARAMETER "object" is null
/// @retval EVDS_ERROR_BAD_PARAMETER "vector" is null
/// @retval EVDS_ERROR_INVALID_OBJECT Object was destroyed
////////////////////////////////////////////////////////////////////////////////
int EVDS_Object_GetInterpolatedStateVector(EVDS_OBJECT* object, EVDS_STATE_VECTOR* vector, EVDS_REAL t) {
	EVDS_STATE_VECTOR v1,v2;
	if (!object) return EVDS_ERROR_BAD_PARAMETER;
	if (!vector) return EVDS_ERROR_BAD_PARAMETER;
#ifndef EVDS_SINGLETHREADED
	if (object->destroyed) return EVDS_ERROR_INVALID_OBJECT;
#endif

	EVDS_Object_GetPreviousStateVector(object,&v1);
	EVDS_Object_GetStateVector(object,&v2);
	EVDS_StateVector_Interpolate(vector,&v1,&v2,t);
	return EVDS_OK;
}

/*int EVDS_Object_StartRendering(EVDS_OBJECT* object, EVDS_STATE_VECTOR* vector) {
	if (!object) return EVDS_ERROR_BAD_PARAMETER;
	if (!vector) return EVDS_ERROR_BAD_PARAMETER;
#ifndef EVDS_SINGLETHREADED
	if (object->destroyed) return EVDS_ERROR_INVALID_OBJECT;
#endif

	memcpy(&object->render_state,vector,sizeof(EVDS_STATE_VECTOR));
	object->render_thread = SIMC_Thread_GetUniqueID();
	return EVDS_OK;
}

int EVDS_Object_EndRendering(EVDS_OBJECT* object) {
	if (!object) return EVDS_ERROR_BAD_PARAMETER;
#ifndef EVDS_SINGLETHREADED
	if (object->destroyed) return EVDS_ERROR_INVALID_OBJECT;
#endif

	object->render_thread = SIMC_THREAD_BAD_ID;
	return EVDS_OK;
}*/


////////////////////////////////////////////////////////////////////////////////
/// @brief Get system from object.
///
/// @param[in] object Object
/// @param[out] p_system Pointer to system, where this object belongs
///
/// @returns Error code, pointer to an EVDS_SYSTEM object
/// @retval EVDS_OK Successfully completed (object matches type)
/// @retval EVDS_ERROR_BAD_PARAMETER "object" is null
/// @retval EVDS_ERROR_BAD_PARAMETER "p_system" is null
/// @retval EVDS_ERROR_INVALID_OBJECT Root object was destroyed
////////////////////////////////////////////////////////////////////////////////
int EVDS_Object_GetSystem(EVDS_OBJECT* object, EVDS_SYSTEM** p_system) {
	if (!object) return EVDS_ERROR_BAD_PARAMETER;
	if (!p_system) return EVDS_ERROR_BAD_PARAMETER;
#ifndef EVDS_SINGLETHREADED
	if (object->destroyed) return EVDS_ERROR_INVALID_OBJECT;
#endif

	*p_system = object->system;
	return EVDS_OK;
}