////////////////////////////////////////////////////////////////////////////////
/// @file
///
/// @brief External Vessel Dynamics Simulator (internal data structures)
/////////////////////////////////////////////////////////////////////////////////
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
#ifndef EVDS_INTERNAL_H
#define EVDS_INTERNAL_H
#ifdef __cplusplus
extern "C" {
#endif

// Internal macro to check functions for errors
#ifdef _DEBUG
#define EVDS_ERRCHECK(expr) { int error_code = expr; EVDS_ASSERT(error_code == EVDS_OK); if (error_code != EVDS_OK) return error_code; }
#else
#define EVDS_ERRCHECK(expr) { int error_code = expr; if (error_code != EVDS_OK) return error_code; }
#endif

// Compatibility with Windows systems
#ifdef _WIN32
#define snprintf _snprintf
#define snscanf _snscanf
#define alloca _alloca
#endif




////////////////////////////////////////////////////////////////////////////////
/// @ingroup EVDS_VARIABLE
/// @struct EVDS_VARIABLE
/// @brief Single variable of an object, or an entry/field in a data structure.
///
/// Variables are used for storing simulation information and objects internal (hidden) state,
/// for example fuel tank dimensions and current amount of fuel in the tank.
///
/// Variables can only be created or removed before the object is initialized. Example:
/// ~~~{.c}
///		EVDS_OBJECT* object;
///		EVDS_VARIABLE* variable;
///		EVDS_Object_Create(system,parent,&object);
///		EVDS_Object_AddVariable(object,"center_of_mass",EVDS_VARIABLE_TYPE_VECTOR,&variable);
///		EVDS_Object_AddRealVariable(object,"mass",5000.0,&variable);
/// ~~~
///
/// If variable already exists when adding, the function simply returns that variable (and does not change
/// its value or create a new one), otherwise creates the variable with the default value.
///
/// @note If variable already exists but has a mismatching variable type, the function for adding variable
///       will fail.
///
/// All work with variables (before the object is initialized) must be done from the initializing thread.
/// If the initalizing thread must be changed in runtime (for example to pass an uninitialized object
/// to another thread) the EVDS_Object_TransferInitialization() call can be used.
///
/// @note Only thread that is initializing/creating the object may change lists of variables,
///       since the raw variable data is not reference counted or tracked in any way. 
///       Any threads which would retrive pointer to variable in other thread may run into
///       crash if variable data is removed.
///
/// A variable may have be of one of the following types:
/// Type								| Description
/// ------------------------------------|--------------------------------------
/// @c EVDS_VARIABLE_TYPE_FLOAT			| Floating point value
/// @c EVDS_VARIABLE_TYPE_STRING		| Null-terminated string
/// @c EVDS_VARIABLE_TYPE_VECTOR		| Vector
/// @c EVDS_VARIABLE_TYPE_QUATERNION	| Quaternion
/// @c EVDS_VARIABLE_TYPE_NESTED		| Data structure (contains a list of nested variables, list of attributes)
/// @c EVDS_VARIABLE_TYPE_DATA_PTR		| Stores pointer to custom data (some C structure)
/// @c EVDS_VARIABLE_TYPE_FUNCTION_PTR	| Stores a function/callback pointer. Function signature depends on variable name
/// @c EVDS_VARIABLE_TYPE_FUNCTION		| Multi-dimensional pre-defined function
///
/// Raw data structure pointers
/// --------------------------------------------------------------------------------
/// It is possible to use variable types @c EVDS_VARIABLE_TYPE_FUNCTION_PTR and @c EVDS_VARIABLE_TYPE_DATA_PTR
/// for storing pointers to arbitrary binary data. These variables cannot be saved or loaded and must only
/// be used in runtime.
///
/// @note Only the pointer is stored, EVDS will not perform any memory management related operations on the pointer.
///
/// @note Custom raw data structures are not thread safe - thread safety must be provided by the user.
///
/// Interpolated functions
/// --------------------------------------------------------------------------------
/// @todo Missing documentation
///
/// Nested variables
/// --------------------------------------------------------------------------------
/// @c EVDS_VARIABLE_TYPE_NESTED type is stored for using arbitrary data structures. It may contain other 
/// nested variables or attributes inside it.
///
/// If the data structure represents some higher level data structure, the solve can create
/// a new variable of EVDS_VARIABLE_TYPE_DATA_PTR type during initialization to store a low-level data
/// structure that will represent data from EVDS_VARIABLE_TYPE_NESTED.
///
/// Here's an example of a nested data structure (geometry information for the tessellator):
/// ~~~{.xml}
///	<parameter name="geometry.cross_sections">
///		<section type="ellipse" add_offset="1" rx="1.9" offset="0.5" />
///		<section type="ellipse" add_offset="1" rx="2" />
///		<section type="ellipse" offset="10.4" add_offset="1" rx="2" />
///		<section type="ellipse" offset="0.4" add_offset="1" rx="1.6" />
///		<section type="ellipse" add_offset="1" rx="1.5" />
///		<section type="ellipse" offset="-0.3" add_offset="1" rx="1.5" />
///	</parameter>
/// ~~~
///
/// Here's an example of a more complicated material entry which contains varied types of
/// custom data:
/// ~~~{.xml}
///	<entry name="Gravimol" class="thermal_soak_shielding">
///	<information>
///		<country>Russia</country>
///		<manufacturer>NII "Graphit", VIAM, NPO "Molniya"</manufacturer>
///	</information>
///	<parameter name="density">1850 kg/m3</parameter>
///	<parameter name="heat_capacity">800 J/(kg K)</parameter>
///	<parameter name="thermal_conductivity">25 W/(m s K)</parameter>
///	<parameter name="melting_point">1925 K</parameter>
///	<parameter name="thermal_expansion_ratio">
///		<data1d x="  20 C">3E-6</data1d>
///		<data1d x="2000 C">5E-6</data1d>
///	</parameter>
///	<parameter name="bend_strength">100 MPa</parameter>
///	<parameter name="compressive_strength">90 MPa</parameter>
///	<parameter name="shear_strength">100 MPa</parameter>
///	<parameter name="tensile_strength">35 MPa</parameter>
///	</entry>
/// ~~~
////////////////////////////////////////////////////////////////////////////////
#ifndef DOXYGEN_INTERNAL_STRUCTS
typedef struct EVDS_VARIABLE_TVALUE_ENTRY_TAG {
	EVDS_REAL x;						//X value
	EVDS_REAL value;					//Constant value
	EVDS_VARIABLE* function;			//Nested function
} EVDS_VARIABLE_TVALUE_ENTRY;

/// Linear interpolation
//#define EVDS_VARIABLE_FUNCTION_INTERPOLATION_LINEAR		0
/// Spline interpolation
//#define EVDS_VARIABLE_FUNCTION_INTERPOLATION_SPLINE		1

typedef struct EVDS_VARIABLE_FUNCTION_TAG {
	//int interpolation;					//Interpolation method for this function
	EVDS_REAL constant_value;				//Constant value of the function
	EVDS_VARIABLE_TVALUE_ENTRY* data;		//Table of values
	int data_count;							//Size of the values table
} EVDS_VARIABLE_FUNCTION;

struct EVDS_VARIABLE_TAG {
#ifndef EVDS_SINGLETHREADED
	SIMC_LOCK_ID lock;						//Read/write lock (only for non-float variables)
#endif

	char name[64];							//Parameter name
	EVDS_VARIABLE_TYPE type;				//Variable type
	void* value;							//Variable value
	size_t value_size;						//Size of variable (size of string if string variable)

	SIMC_LIST* attributes;					//Attributes of this variable (for arbitrary data only)
	SIMC_LIST* list;						//List of nested variables
	SIMC_LIST_ENTRY* attribute_entry;		//Entry in parent variables attributes list
	SIMC_LIST_ENTRY* list_entry;			//Entry in parent variables nested list (or objects variables list)

	EVDS_VARIABLE* parent;					//Variable this variable belongs to (if nested)
	EVDS_OBJECT* object;					//Object this parameter belongs to (0 if not a parameter)
	EVDS_SYSTEM* system;					//System this variable belongs to

	// User-defined data
	void* userdata;
};
#endif




////////////////////////////////////////////////////////////////////////////////
/// @ingroup EVDS_OBJECT
/// @struct EVDS_OBJECT
/// @brief Single object with a coordinate system.
///
/// EVDS objects can be used to define:
///  - Propagators (coordinate systems in which children states are propagated)
///  - Vessels/moving objects
///  - Planets and planet-like bodies
///  - Objects which create forces (engines, wings)
///  - Special objects which may create additional objects during initialization
///
/// The objects behavior depends on its parent. Some objects are only meaningful
/// when their parent is some specific object type (for example engine nozzles will
/// only affect behavior of the engine object, and do not have any behavior otherwise).
///
/// @note The object hierarchy as saved to file may not match up with what the simulation
///       will use for calculations - additional objects may be added, some may be removed.
///
/// From EVDS point of view the objects can be in three states:
///  - Created (after object was just created) - this allows setting up the objects internal
///    variables.
///  - Initializing (while object is being initialized by solvers initializer)
///  - Active/initialized - when object becomes part of the physical world.
///
/// Every EVDS object may have some basic dynamic data defined:
///  - Mass
///  - Moments of inertia tensor
///  - Center of mass position (without accounting for children objects)
///  - Cross-sections that define objects geometry
///
/// This information is optional and will only be used for representing the physical shape
/// and properties of the object. Only some specific solvers (for example rigid body) will
/// use this information for actual physics.
///
/// Each object contains a list of variables (see EVDS_VARIABLE). These variables represent
/// objects internal (hidden) state and various object parameters.
///
/// @note Only thread that is initializing/creating the object may change lists of variables,
///       since the raw variable data is not reference counted or tracked in any way. 
///       Any threads which would retrive pointer to variable in other thread may run into
///       crash if variable data is removed.
///
/// Objects are defined by their type and name. Two objects may not have the same name within
/// the same parent while they are being initialized.
/// Two objects may share names globally, but only one of them will be returned when queried.
///
/// @note If two names do in fact have same names under one parent, they will be renamed during initialization.
///
/// Each object additionally has a numerical identifier that's unique globally. It is used for
/// identifying objects in a shorter way and also for networking facilities. It can be set by user
/// before the object is initialized.
///
/// @note If initialized object has a unique identifier that's already used, it will be given a new
///       unique identifier.
///
/// See EVDS_Object_SetName(), EVDS_Object_SetType(), EVDS_Object_SetUID() for more information
/// about object names, types, unique identifiers.
///
/// Object may contain a pointer to user data (any data structure user wishes to attach to the object),
/// and a pointer to solver data (any data structure that was assigned to the object by the solver).
/// API is same for both pointers, but the solver data must be used exclusively by the objects solver:
/// ~~~{.c}
///		void* userdata = malloc(128);
///		EVDS_Object_SetUserdata(object,userdata);
/// ~~~
/// ~~~{.c}
///		void* userdata;
///		EVDS_Object_GetUserdata(object,&userdata);
///		free(userdata);
/// ~~~
///
/// EVDS contains fast partially-lockless multithreading support for the objects. The following limits apply:
///  - Objects can be created and destroyed in any threads at any time
///  - Application must stop using destroyed objects within moments after they have been destroyed.
///  - If application has to keep object data around for longer, then objects must be reference counted
///    explicitly by the application using EVDS_Object_Store() and EVDS_Object_Release().
///  - A single object must not be deleted from more than one thread at once (this will cause race condition
///    and the EVDS system will enter an interdeterminate state)
///  - Object data will be retained in memory until no more references to it exist,
///     and physically removed after EVDS_System_CleanupObjects() is called
///  - Objects must be initialized after loading. Initialization and all operations which
///     alter list of variables must be done only within the initializing thread (operations
///     with list of variables are not thread-safe).
////////////////////////////////////////////////////////////////////////////////
#ifndef DOXYGEN_INTERNAL_STRUCTS
struct EVDS_OBJECT_TAG {
	//Unique ID (numeric identifier for the object)
	unsigned int uid;						//00000 - 99999 reserved for normal vessels

	// Object coordinates and state in space
	EVDS_STATE_VECTOR state;
	// Previous state vector (used for interpolation when rendering)
	EVDS_STATE_VECTOR previous_state;
	// State in which object must be rendered
	EVDS_STATE_VECTOR render_state;			//FIXME

	// Public object state, used by functions which are not the integrating thread
#ifndef EVDS_SINGLETHREADED
	SIMC_SRW_ID state_lock;
	SIMC_SRW_ID previous_state_lock;
	EVDS_STATE_VECTOR private_state;
	SIMC_THREAD_ID integrate_thread;		//Thread that's integrating position 
											// (makes transformations use "state" and not "public_state")
	SIMC_THREAD_ID render_thread;			//Rendering thread (overrides coordinate conversions)
#endif

	// Fixed object information
	char name[256];							//Object name
	char type[256];							//Object type
	EVDS_OBJECT* parent;					//Objects parent
	EVDS_SOLVER* solver;					//Objects solver
	EVDS_SYSTEM* system;					//Objects system
	int parent_level;						//How many nodes away from root (0 for root)

	// Object variables/parameters
	SIMC_LIST* variables;					//List of variables
	SIMC_LIST* children;					//Children objects
	SIMC_LIST* raw_children;				//Children objects (raw list, including the uninitialized ones)

	// Initialization-related information
	int initialized;						//Is object initialized
#ifndef EVDS_SINGLETHREADED
	SIMC_THREAD_ID initialize_thread;		//Thread that performs initialization
	SIMC_THREAD_ID create_thread;			//Thread in which object was created
#endif

	// Information for destroying the object
#ifndef EVDS_SINGLETHREADED
	int stored_counter;						//Instance counter (how many times object was stored elsewhere)
	int destroyed;							//Object is destroyed and must be removed from storage ASAP
#endif
	SIMC_LIST_ENTRY* object_entry;			//Entry in "system->objects" linked list (used for removing it from list)
	SIMC_LIST_ENTRY* parent_entry;			//Entry in "parent->children" linked list (used for removing it from list)
	SIMC_LIST_ENTRY* rparent_entry;			//Entry in "parent->raw_children" linked list (used for removing it from list)
	SIMC_LIST_ENTRY* type_entry;			//Entry in "type_list" linked list (used for removing it from list)
	SIMC_LIST* type_list;					//List in which type is stored (or 0)

	// Callbacks
	EVDS_Callback_Solve*		solve;		//Solve object/step state forward
	EVDS_Callback_Integrate*	integrate;	//Return derivative of state vector for integration

	// User-defined data
	void* userdata;
	void* solverdata;
};
#endif




////////////////////////////////////////////////////////////////////////////////
/// @ingroup EVDS_SYSTEM
/// @struct EVDS_SYSTEM
/// @brief Object that represents a self-contained system of objects.
///
/// The EVDS system represents one instance of the simulator. It contains a tree
/// of EVDS_OBJECT data structures (which represent objects and coordinate systems),
/// and a set of EVDS_SOLVER data structures (which represent the different object types
/// and object behaviors).
///
/// EVDS system always contains a single "root" object, which represents the inertial space
/// of the simulated universe. All objects are either children of root object or other objects.
///
/// The tree of objects must not contain any loops. The API does not provide checks against loops,
/// creating a loop (e.g. root node is a child of an object that's a child of root node) will put
/// the program into undefined state.
///
/// It is possible to copy objects between two EVDS_SYSTEM objects, see EVDS_Object_Copy() and
/// EVDS_Object_CopySingle().
///
/// @note All application threads still working with EVDS objects must stop before EVDS_SYSTEM is
///       destroyed, otherwise the data may be deleted while threads are still busy.
///
/// Destroying the EVDS_SYSTEM object will clean up all relevant data, including
/// all EVDS_OBJECT data structures. The following data will not be cleaned up:
///  - Pointers to custom data structure variables within objects (must be cleaned up by solvers)
///  - All userdata objects must be cleaned up manually by their user
////////////////////////////////////////////////////////////////////////////////
#ifndef DOXYGEN_INTERNAL_STRUCTS
typedef struct EVDS_INTERNAL_TYPE_ENTRY_TAG {
	char type[256];			//Type name
	SIMC_LIST* objects;		//List of objects with this type
} EVDS_INTERNAL_TYPE_ENTRY;

struct EVDS_SYSTEM_TAG {
	// Object data management
#ifndef EVDS_SINGLETHREADED
	SIMC_LOCK_ID cleanup_working;				// Delete thread working
	SIMC_LIST* deleted_objects;					// List of deleted objects
#endif
	SIMC_LIST* objects;							// List of objects
	SIMC_LIST* object_types;					// List of object types

	// Other lists
	SIMC_LIST* solvers;							// List of solvers
	SIMC_LIST* databases;						// List of databases (each an EVDS_VARIABLE)

	// Global callbacks
	EVDS_Callback_Initialize* OnInitialize;		// Global initialization callback

	// Various special variables
	unsigned int uid_counter;					// Unique ID counter (for objects without a defined UID)
	EVDS_OBJECT* inertial_space;				// Root inertial space
	EVDS_REAL time;								// Global system time

	// User-defined data
	void* userdata;
};
#endif




////////////////////////////////////////////////////////////////////////////////
/// @ingroup EVDS_MESH
/// @struct EVDS_MESH_INTERNAL
/// @brief Internal data used by EVDS_Mesh_GenerateEx() during mesh generation.
///
/// This data structure is used internally by the mesh tessellator to hold the intermediate
/// tessellation results, various temporary information.
////////////////////////////////////////////////////////////////////////////////
#ifndef DOXYGEN_INTERNAL_STRUCTS
struct EVDS_MESH_INTERNAL_TAG {
	SIMC_STORAGEARRAY* vertices;			///< Temporary storage for vertices
	SIMC_STORAGEARRAY* indices;				///< Temporary storage for indices
	SIMC_STORAGEARRAY* vertex_info;			///< Temporary storage for vertex information
	SIMC_STORAGEARRAY* triangles;			///< Temporary storage for triangles
	int max_smoothing_group;				///< Maximum index of a smoothing group
};
#endif




////////////////////////////////////////////////////////////////////////////////
// Internal API
////////////////////////////////////////////////////////////////////////////////
// Destroy object internal data
int EVDS_InternalObject_DestroyData(EVDS_OBJECT* object);
// Destroy variable internal data
int EVDS_InternalVariable_DestroyData(EVDS_VARIABLE* variable);
// Creates a new variable
int EVDS_Variable_Create(EVDS_SYSTEM* system, const char* name, EVDS_VARIABLE_TYPE type, EVDS_VARIABLE** p_variable);
// Creates a new variable as a copy of existing one
int EVDS_Variable_Copy(EVDS_VARIABLE* source, EVDS_VARIABLE* variable);
// Initialize function data
int EVDS_InternalVariable_InitializeFunction(EVDS_VARIABLE* variable, EVDS_VARIABLE_FUNCTION* function, const char* data);
// Destroy function data
int EVDS_InternalVariable_DestroyFunction(EVDS_VARIABLE* variable, EVDS_VARIABLE_FUNCTION* function);

#ifndef EVDS_SINGLETHREADED
// Set private state vector
int EVDS_InternalObject_SetPrivateStateVector(EVDS_OBJECT* object, EVDS_STATE_VECTOR* vector);
#endif

#ifdef __cplusplus
}
#endif
#endif
