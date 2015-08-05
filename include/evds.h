////////////////////////////////////////////////////////////////////////////////
/// @file
///
/// @brief External Vessel Dynamics Simulator
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
#ifndef EVDS_H
#define EVDS_H
#ifdef __cplusplus
extern "C" {
#endif


////////////////////////////////////////////////////////////////////////////////
// Library management
////////////////////////////////////////////////////////////////////////////////
#define EVDS_VERSION			38
#ifndef EVDS_DYNAMIC
#	define EVDS_API
#else
#	ifdef EVDS_LIBRARY
#		define EVDS_API __declspec(dllexport)
#	else
#		define EVDS_API __declspec(dllimport)
#	endif
#endif

#ifdef _DEBUG
#	define EVDS_ERRCHECK(expr) { int error_code = expr; EVDS_ASSERT(error_code == EVDS_OK); if (error_code != EVDS_OK) return error_code; }
#	define EVDS_ASSERT(what) ((what) ? ((void)0) : EVDS_Log(EVDS_ERROR,"Assert failed: %s (%s:%d)\n",#what,__FILE__,__LINE__))
#	ifdef PLATFORM32
#		define EVDS_BREAKPOINT() _asm {int 3}
#	else
#		define EVDS_BREAKPOINT() __debugbreak()
#	endif
#else
#	define EVDS_ERRCHECK(expr) { int error_code = expr; if (error_code != EVDS_OK) return error_code; }
#	define EVDS_ASSERT(nothing) ((void)0)
#	define EVDS_BREAKPOINT()
#endif

#include "stddef.h"
#include "sim_core.h"




////////////////////////////////////////////////////////////////////////////////
/// @defgroup EVDS_BASIC Basic Definitions
/// @brief List of basic types, macros, callbacks used in EVDS.
///
/// @{
////////////////////////////////////////////////////////////////////////////////

/// Real number used for all mathematical operations in EVDS
typedef double EVDS_REAL;
/// EVDS variable type
typedef unsigned int EVDS_VARIABLE_TYPE;

/// Smallest meaningful number represented by EVDS_REAL
#define EVDS_EPS 1e-15
/// Smallest meaningful number represented by a single-precision float
#define EVDS_EPSf 1e-6f
/// Largest meaningful number represented by EVDS_REAL
#define EVDS_INFINITY 1e308
/// Largest meaningful number represented by a single-precision float
#define EVDS_INFINITYf 1e37f

/// Standard gravity
#define EVDS_G0	9.80665
/// Speed of light
#define EVDS_C 299792458
/// Boltzman constant
#define EVDS_K 1.3806488e-23

/// PI constant
#define EVDS_PI 3.14159265358979323846264338327950288419716939937510
/// PI constant (floating point)
#define EVDS_PIf 3.14159265358979323846f
/// Macro to convert angle from degrees to radians
#define EVDS_RAD(x) (EVDS_PI*(x)/180.0)
/// Macro to convert angle from radians to degrees
#define EVDS_DEG(x) (180.0*(x)/EVDS_PI)

/// Special MJD date that means EVDS system must run realtime
#define EVDS_REALTIME -1.0

/// Information message
#define EVDS_INFO		0
/// Warning message
#define EVDS_WARNING	1
/// Error message
#define EVDS_ERROR		2
/// Information/message callback
typedef int EVDS_Callback_Log(int type, char* message);

////////////////////////////////////////////////////////////////////////////////
/// @}
////////////////////////////////////////////////////////////////////////////////




// Forward structure declarations
typedef struct EVDS_VARIABLE_TAG EVDS_VARIABLE;
typedef struct EVDS_OBJECT_TAG EVDS_OBJECT;
typedef struct EVDS_SYSTEM_TAG EVDS_SYSTEM;
typedef struct EVDS_MODIFIER_TAG EVDS_MODIFIER;
typedef struct EVDS_SOLVER_TAG EVDS_SOLVER;
typedef struct EVDS_OBJECT_LOADEX_TAG EVDS_OBJECT_LOADEX;
typedef struct EVDS_OBJECT_SAVEEX_TAG EVDS_OBJECT_SAVEEX;
typedef struct EVDS_MESH_GENERATEEX_TAG EVDS_MESH_GENERATEEX;
typedef struct EVDS_MESH_INTERNAL_TAG EVDS_MESH_INTERNAL;




////////////////////////////////////////////////////////////////////////////////
/// @ingroup EVDS_MATH
/// @brief Stores a 3D vector and its type, position in space, first derivative of position.
///
/// It is possible to directly access the numerical values of a vector, but it is advised to only
/// work with vectors through the vector API to invoke the built-in EVDS library sanity checks and
/// automatic conversion mechanisms.
///
/// All vectors are numerically defined in their coordinate systems. All units of measurements are
/// SI.
///
/// Vector operations will only accept same-type vectors. The sanity checking mechanism is only
/// enabled in debug mode via asserts. The checks are omited in the release build for perfomance reasons.
///
/// The following vector types are currently defined:
///  Type								| Units		| Description
/// ----------------------------------- | ----------|-------------
/// @c EVDS_VECTOR_DIRECTION			| None		| Direction vector
/// @c EVDS_VECTOR_DISPLACEMENT			| m			| Position displacement vector
/// @c EVDS_VECTOR_FORCE				| N			| Force vector (same type as the direction vector)
/// @c EVDS_VECTOR_TORQUE				| N m		| Torque vector
/// @c EVDS_VECTOR_POSITION				| m			| Position in 3D space
/// @c EVDS_VECTOR_VELOCITY				| m/s		| Velocity
/// @c EVDS_VECTOR_ACCELERATION			| m/s^2		| Acceleration
/// @c EVDS_VECTOR_ANGULAR_VELOCITY		| rad/s		| Angular velocity
/// @c EVDS_VECTOR_ANGULAR_ACCELERATION	| rad/s2	| Angular acceleration
/// @c EVDS_VECTOR_INERTIAL_TRANSFORM	| N/A		| Special vector type to indicate that this vector must be transformed without non-inertial components
///
/// @note Direction vectors share the vector type with force vectors - so a force vector can be treated as
///       directional one or vice versa without conversion.
///
/// Automatic conversion between these types is possible by some of the functions.
/// Refer to documentation of the vector/math API for more specific information, for example
/// EVDS_Vector_Cross().
///
/// @note Vectors passed into math library as operands must always be initialized.
///
/// For all vector operations, the resulting vector is in the coordinate system of the first operand:
/// ~~~{.c}
///		EVDS_VECTOR v1;
///		EVDS_VECTOR v2;
///		EVDS_VECTOR result;
///		EVDS_Vector_Add(&result,&v1,&v2); //Result in coordinate system of v1
/// ~~~
///
/// State vectors (see EVDS_STATE_VECTOR) always have their components defined in the coordinate
/// system of the parent object (to which the state vector belongs).
////////////////////////////////////////////////////////////////////////////////
typedef struct EVDS_VECTOR_TAG {
	EVDS_REAL x;								///< X vector component
	EVDS_REAL y;								///< Y vector component
	EVDS_REAL z;								///< Z vector component
	int derivative_level;						///< Derivative/vector type
	EVDS_OBJECT* coordinate_system;				///< Object in whose coordinates this vector is defined

	EVDS_REAL px;								///< X component of location of this vector in space
	EVDS_REAL py;								///< Y component of location of this vector in space
	EVDS_REAL pz;								///< Z component of location of this vector in space
	EVDS_OBJECT* pcoordinate_system;			///< Coordinate system of position vector

	EVDS_REAL vx;								///< X component of velocity of this vector in space
	EVDS_REAL vy;								///< Y component of velocity of this vector in space
	EVDS_REAL vz;								///< Z component of velocity of this vector in space
	EVDS_OBJECT* vcoordinate_system;			///< Coordinate system of velocity vector
} EVDS_VECTOR;


////////////////////////////////////////////////////////////////////////////////
/// @ingroup EVDS_MATH
/// @brief Stores a 3D vector and no references to derivative or coordinate system
////////////////////////////////////////////////////////////////////////////////
typedef struct EVDS_SHORT_VECTOR_TAG {
	EVDS_REAL x;								///< X vector component
	EVDS_REAL y;								///< Y vector component
	EVDS_REAL z;								///< Z vector component
} EVDS_SHORT_VECTOR;


////////////////////////////////////////////////////////////////////////////////
/// @ingroup EVDS_MATH
/// @brief Represents an orientation (attitude) of an object in the given coordinate system.
///
/// The stored quaternion is represented as \f$q_0 + q_1 i + q_2 j + q_3 k\f$
/// where \f$q_n\f$ are members of q[] array in EVDS_QUATERNION.
/// 
/// Quaternion operations have special rules related to the coordinate systems,
/// for example EVDS_Quaternion_Multiply() requires two quaternions in same coordinate
/// system OR one quaternion being a child of another. Refer to quaternion API documentation
/// for more detailed information.
///
/// State vectors (see EVDS_STATE_VECTOR) always have their components defined in the coordinate
/// system of the parent object (to which the state vector belongs).
////////////////////////////////////////////////////////////////////////////////
typedef struct EVDS_QUATERNION_TAG {
	EVDS_REAL q[4];								///< Quaternion real and imaginary parts
	EVDS_OBJECT* coordinate_system;				///< Object in whose coordinates this quaternion is defined
} EVDS_QUATERNION;


////////////////////////////////////////////////////////////////////////////////
/// @ingroup EVDS_MATH
/// @brief Represents a 4x4 matrix (row-major, right-handed).
///
/// EVDS_QUATERNION can be converted to EVDS_MATRIX using the EVDS_Quaternion_ToMatrix() API call.
/// EVDS matrix is defined in C order:
/// \f[
///		M = \left| \begin{array}{cccc} 
///			m_{0} & m_{1} & m_{2} & m_{3} \\ 
///			m_{4} & m_{5} & m_{6} & m_{7} \\ 
///			m_{8} & m_{9} & m_{10} & m_{11} \\ 
///			m_{12} & m_{13} & m_{14} & m_{15}
///		\end{array} \right|
/// \f]
///
/// Matrices do not have a coordinate system assigned to them, but they can be assumed to
/// have the same coordinate system as the quaternion, from which the matrix was generated.
////////////////////////////////////////////////////////////////////////////////
typedef EVDS_REAL EVDS_MATRIX[4*4];


////////////////////////////////////////////////////////////////////////////////
/// @ingroup EVDS_MATH
/// @brief A state vector that fully defines current visible state of the vessel.
///
/// The state vector represents visible state of the objects - position, velocity, etc.
/// It does not contain hidden information (internal state of the vessel).
///
/// Acceleration and angular acceleration are not directly part of the state vector,
/// but they are values of derivatives at time of this state vector.
////////////////////////////////////////////////////////////////////////////////
typedef struct EVDS_STATE_VECTOR_TAG {
	// Last time state vector was updated
	double time;						///< This states time, MJD
	// Translation
	EVDS_VECTOR position;				///< Current position
	EVDS_VECTOR velocity;				///< Current velocity
	EVDS_VECTOR acceleration;			///< Current acceleration (only for information)
	// Orientation
	EVDS_QUATERNION orientation;		///< Current orientation quaternion
	EVDS_VECTOR angular_velocity;		///< Current angular velocity
	EVDS_VECTOR angular_acceleration;	///< Current angular acceleration (only for information)
	// User-defined components (not supported yet)
	//int userdata_count;
	//EVDS_REAL* userdata;
} EVDS_STATE_VECTOR;


////////////////////////////////////////////////////////////////////////////////
/// @ingroup EVDS_FRAME
/// @brief Derivative of the EVDS_STATE_VECTOR.
///
/// Describes derivative at the time specified in the corresponding EVDS_STATE_VECTOR.
///
/// Only contains the state vector derivative, may also contain information about
/// force or torque. Used in propagators.
////////////////////////////////////////////////////////////////////////////////
typedef struct EVDS_STATE_VECTOR_DERIVATIVE_TAG {
	// Translation
	EVDS_VECTOR velocity;				///< Current velocity
	EVDS_VECTOR acceleration;			///< Current acceleration
	// Orientation
	EVDS_VECTOR angular_velocity;		///< Current angular velocity
	EVDS_VECTOR angular_acceleration;	///< Current angular acceleration
	// Force/torque (alternative to acceleration/angular acceleration)
	EVDS_VECTOR force;					///< Current force
	EVDS_VECTOR torque;					///< Current torque
	// User-defined components (not supported yet)
	//int userdata_count;
	//EVDS_REAL* userdata_derivative;
} EVDS_STATE_VECTOR_DERIVATIVE;


////////////////////////////////////////////////////////////////////////////////
/// @ingroup EVDS_FRAME
/// @brief Geodetic datum.
///
/// Represents a geodetic datum for use with the geographic coordinate conversion
/// functions. EVDS currently only supports planetwide geodetic datums, represented
/// by an ellipsoid centered in the planets reference point (center of mass).
///
/// The ellipsoid is defined by semimajor, semiminor axes and references the planetary
/// body object. The datum can be derived from parameters of the celestial body, see
/// EVDS_Geodetic_DatumFromObject().
///
/// This data structure may be extended in the future if any additional geodetic
/// datum features are required.
////////////////////////////////////////////////////////////////////////////////
typedef struct EVDS_GEODETIC_DATUM_TAG {
	EVDS_OBJECT* object;		///< Object around which the coordinate is specified
	EVDS_REAL semimajor_axis;	///< Semi-major axis of the planet
	EVDS_REAL semiminor_axis;	///< Semi-minor axis of the planet
} EVDS_GEODETIC_DATUM;


////////////////////////////////////////////////////////////////////////////////
/// @ingroup EVDS_MATH
/// @brief Geodetic coordinate (latitude, longitude, elevation).
///
/// This data structure represents geodetic coordinates around a body. The coordinates are
/// specified using a datum specified explicitly or derived from planetary body parameters.
///
/// @note Datum must be correctly specified for EVDS_Geodetic_ToVector() function call. See
///       EVDS_Geodetic_Set() for defining geodetic coordinates and automatically calculating datum.
///
/// For non-celestial bodies the geodetic coordinates will correspond to the following
/// coordinates with origin around objects reference point:
/// Geodetic coordinate			| Vessel-relative coordinate
///	----------------------------|-------------------------------
///	Latitude					| Relative elevation
///	Longitude					| Relative bearing
/// Elevation					| Relative distance
///
/// The geodetic coordinates have the following valid ranges:
/// Geodetic coordinate			| Units		| Range
///	----------------------------|-----------|-------------------------
///	Latitude					| degrees	| \f$[-90.0, 90.0]\f$
///	Longitude					| degrees	| \f$[-180.0, 180.0)\f$
/// Elevation					| m			| \f$[0.0, \infty)\f$
///
/// @note Geodetic longitude \f$180.0 \ degrees\f$ is not represented in EVDS_GEODETIC_COORDINATE.
///       Longitude of \f$-180.0 \ degrees\f$ is used instead.
///
////////////////////////////////////////////////////////////////////////////////
typedef struct EVDS_GEODETIC_COORDINATE_TAG {
	EVDS_REAL latitude;				///< Geodetic latitude (relative elevation)
	EVDS_REAL longitude;			///< Geodetic longitude (relative bearing)
	EVDS_REAL elevation;			///< Elevation (relative distance)
	EVDS_GEODETIC_DATUM datum;		///< Datum in which coordinate is specified
} EVDS_GEODETIC_COORDINATE;




////////////////////////////////////////////////////////////////////////////////
/// @ingroup EVDS_SOLVER
/// @{

/// Called when object is being initialized
typedef int EVDS_Callback_Initialize(EVDS_SYSTEM* system, EVDS_SOLVER* solver, EVDS_OBJECT* object);
/// Called after the object was initialized (global callback)
typedef int EVDS_Callback_PostInitialize(EVDS_SYSTEM* system, EVDS_SOLVER* solver, EVDS_OBJECT* object);
/// Called when object is being deinitialized
typedef int EVDS_Callback_Deinitialize(EVDS_SYSTEM* system, EVDS_SOLVER* solver, EVDS_OBJECT* object);
/// Called when object data is being destroyed (delayed destroy) to cleanup data which may exist between object being destroyed in system and for real
typedef int EVDS_Callback_Finalize(EVDS_SYSTEM* system, EVDS_OBJECT* object);
/// Called when solver is started up
typedef int EVDS_Callback_Startup(EVDS_SYSTEM* system, EVDS_SOLVER* solver);
/// Called when solver is shut down
typedef int EVDS_Callback_Shutdown(EVDS_SYSTEM* system, EVDS_SOLVER* solver);
/// Called when objects state must be saved (FIXME: Not implemented)
typedef int EVDS_Callback_StateSave(EVDS_SYSTEM* system, EVDS_SOLVER* solver, EVDS_OBJECT* object);
/// Called when objects state must be restored (FIXME: Not implemented)
typedef int EVDS_Callback_StateLoad(EVDS_SYSTEM* system, EVDS_SOLVER* solver, EVDS_OBJECT* object);
/// "Solve" callback (propagate objects state by delta_time). Can update "object"
typedef int EVDS_Callback_Solve(EVDS_SYSTEM* system, EVDS_SOLVER* solver, EVDS_OBJECT* object,
								EVDS_REAL delta_time);
/// "Integrate" callback (set "derivative" based on "state" and "time"). Must not update "object" state
typedef int EVDS_Callback_Integrate(EVDS_SYSTEM* system, EVDS_SOLVER* solver, EVDS_OBJECT* object,
									EVDS_REAL delta_time, EVDS_STATE_VECTOR* state, EVDS_STATE_VECTOR_DERIVATIVE* derivative);

/// @}
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
/// @ingroup EVDS_SOLVER
/// @brief Definition of a physics solver.
///
/// A physics solver is a sub-simulation program responsible for calculating a certain
/// specific kind of object. For example the "rigid body" solver calculates rigid body physics,
/// accumulates forces and outputs accelerations for the rigid body.
///
/// The solvers must be registered with EVDS_Solver_Register() API call. Only
/// solvers that have been registered before an object was created will be used.
///
/// A solver may omit some of its callbacks. The userdata is usually defined by
/// the solver during its startup.
/// The EVDS_SOLVER::OnInitialize callback is required for the solver to be able
/// to accepts objects during initialization.
/// 
/// @note The initialization callback must return EVDS_CLAIM_OBJECT or EVDS_IGNORE_OBJECT. It will be
///       called for every object created and initialized with EVDS_SYSTEM.
///
/// The EVDS_SOLVER::OnStartup callback is called right after the solver was registered with EVDS_SYSTEM.
/// EVDS_SOLVER::OnShutdown callback will be called when EVDS_SYSTEM is destroyed. The solver may allocate and free
/// its 'global' resources in these callbacks.
///
/// A quick example for a simple solver that does nothing useful:
/// ~~~{.c}
///		int EVDS_MetalRod_Solve(EVDS_SYSTEM* system, EVDS_SOLVER* solver, EVDS_OBJECT* object, EVDS_REAL delta_time) {
///			return EVDS_OK;
///		}
///		
///		int EVDS_MetalRod_Initialize(EVDS_SYSTEM* system, EVDS_SOLVER* solver, EVDS_OBJECT* object) {
///			if (EVDS_Object_CheckType(object,"my_ext.metal_rod") != EVDS_OK) return EVDS_IGNORE_OBJECT; 
///			return EVDS_CLAIM_OBJECT;
///		}
///		
///		EVDS_SOLVER EVDS_Solver_MetalRod = {
///			EVDS_MetalRod_Initialize, //OnInitialize
///			0, //OnDeinitialize
///			EVDS_MetalRod_Solve, //OnSolve
///			0, //OnIntegrate
///			0, //OnStateSave
///			0, //OnStateLoad
///			0, //OnStartup
///			0, //OnShutdown
///		};
////
///		int EVDS_MetalRod_Register(EVDS_SYSTEM* system) {
///			return EVDS_Solver_Register(system,&EVDS_Solver_MetalRod);
///		}
/// ~~~
///
////////////////////////////////////////////////////////////////////////////////
#ifndef DOXYGEN_INTERNAL_STRUCTS
struct EVDS_SOLVER_TAG {
#else
struct EVDS_SOLVER {
#endif
	// Callbacks
	EVDS_Callback_Initialize*		OnInitialize;		///< Called when object is being initialized
	EVDS_Callback_Deinitialize*		OnDeinitialize;		///< Called when object is being deinitialized
	EVDS_Callback_Solve*			OnSolve;			///< Called to propagate object state in time
	EVDS_Callback_Integrate*		OnIntegrate;		///< Called to calculate derivative from objects state vector
	EVDS_Callback_StateSave*		OnStateSave;		///< Called when objects state must be saved
	EVDS_Callback_StateLoad*		OnStateLoad;		///< Called when objects state must be restored

	// Called only once
	EVDS_Callback_Startup*			OnStartup;			///< Called when solver is started up
	EVDS_Callback_Shutdown*			OnShutdown;			///< Called when solver is shut down

	// Extra callbacks
	EVDS_Callback_Finalize*			OnFinalize;			///< Called when object data is deleted

	// User-defined data
	void* userdata;										///< Pointer to user data
};


////////////////////////////////////////////////////////////////////////////////
/// @ingroup EVDS_SYSTEM
/// @brief A list of global callbacks.
///
/// See EVDS_SOLVER for more information on how the callbacks are used. This data
/// structure allows setting a list of global callbacks for EVDS system.
///
/// The system-wide callback will be called before every object is initialized. It can be used to append
/// additional simulator-specific variables of interest. The callback will be called from the initializing
/// thread (which may be different from the one that called EVDS_Object_Initialize()).
///
/// The system-wide callback must return EVDS_OK if completed successfully. EVDS_CLAIM_OBJECT can be returned
/// to claim the object (the solvers initialization routine will then be ignored).
////////////////////////////////////////////////////////////////////////////////
typedef struct EVDS_GLOBAL_CALLBACKS_TAG {
	//Callbacks
	EVDS_Callback_Initialize*		OnInitialize;		///< Called when object is being initialized
	EVDS_Callback_Deinitialize*		OnDeinitialize;		///< Called when object is being deinitialized
	EVDS_Callback_PostInitialize*	OnPostInitialize;	///< Called after the object was initialized
	//EVDS_Callback_StateSave*		OnStateSave;		///< Called when objects state must be saved
	//EVDS_Callback_StateLoad*		OnStateLoad;		///< Called when objects state must be restored
	//EVDS_Callback_Startup*			OnStartup;			///< Called when solver is started up
	//EVDS_Callback_Shutdown*			OnShutdown;			///< Called when solver is shut down
} EVDS_GLOBAL_CALLBACKS;




////////////////////////////////////////////////////////////////////////////////
/// @ingroup EVDS_OBJECT
/// @{

/// Called for every loaded object (see EVDS_Object_LoadFromFileEx())
typedef int EVDS_Callback_LoadObject(EVDS_OBJECT_LOADEX* info, EVDS_OBJECT* object);
/// Called when syntax error happens
typedef int EVDS_Callback_SyntaxError(EVDS_OBJECT_LOADEX* info, const char* error);

/// Do not automatically execute modifiers
#define EVDS_OBJECT_LOADEX_SKIP_MODIFIERS		1
/// Load only first object
#define EVDS_OBJECT_LOADEX_ONLY_FIRST			2
/// Do not load objects from the file
#define EVDS_OBJECT_LOADEX_NO_OBJECTS			4
/// Do not load modifiers from the file
//#define EVDS_OBJECT_LOADEX_NO_MODIFIERS			8
/// Do not load databases from the file
#define EVDS_OBJECT_LOADEX_NO_DATABASES			16
/// Do not load solver states
//#define EVDS_OBJECT_LOADEX_NO_SOLVER_STATE	32
/// Load metadata objects (FIXME: add support for this)
#define EVDS_OBJECT_LOADEX_LOAD_METADATA		64
/// Do not initialize objects when loading
#define EVDS_OBJECT_LOADEX_DONT_INITIALIZE		128
/// Use blocking initialization
#define EVDS_OBJECT_LOADEX_BLOCKING_INITIALIZE	256

/// Save only children of the object passed into EVDS_Object_SaveEx()
#define EVDS_OBJECT_SAVEEX_ONLY_CHILDREN		1
/// Append objects to existing file or description
//#define EVDS_OBJECT_SAVEEX_APPEND_OBJECTS		2
/// Save all modified objects as if they were normal objects
#define EVDS_OBJECT_SAVEEX_SAVE_COPIES			4
/// Do not save modifiers
//#define EVDS_OBJECT_SAVEEX_SKIP_MODIFIERS		8
/// Save unique ID's of objects
#define EVDS_OBJECT_SAVEEX_SAVE_UIDS			16
/// Save full state vector for object (including time and quaternion, solver states)
#define EVDS_OBJECT_SAVEEX_SAVE_FULL_STATE		32

/// @}
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
/// @ingroup EVDS_OBJECT
/// @brief Information structure for EVDS_Object_LoadEx().
///
/// Additional callbacks may be specified that will be called during loading:
///	Name				| Description
///	--------------------|--------------------------
///	@c OnLoadObject		| Called for every loaded object.
/// @c OnSyntaxError	| Called when a parsing syntax error occurs. A human-readable error description will be passed.
///
/// If EVDS_OBJECT_LOADEX::description is set within the structure, information will be loaded from
/// the description string instead of the filename.
///
/// If a syntax error occurs, the syntax error callback will be called just once for
/// the first error in file.
///
/// Additional flags can be specified to change default behavior of the loader:
/// Name									| Description
/// ----------------------------------------|----------------------
/// @c EVDS_OBJECT_LOADEX_ONLY_FIRST		| Only load the first object in file and skip all following.
/// @c EVDS_OBJECT_LOADEX_NO_OBJECTS		| Do not load any objects from the file.
/// @c EVDS_OBJECT_LOADEX_NO_DATABASES		| Do not load any databases from the file.
///
///	See EVDS_Object_LoadEx() for more information.
////////////////////////////////////////////////////////////////////////////////
#ifndef DOXYGEN_INTERNAL_STRUCTS
struct EVDS_OBJECT_LOADEX_TAG {
#else
struct EVDS_OBJECT_LOADEX {
#endif
	EVDS_Callback_LoadObject*	OnLoadObject;	///< Called for every loaded object
	EVDS_Callback_SyntaxError*	OnSyntaxError;	///< Called when a syntax error occurs

	int							version;		///< Version of loaded file is written here
	int							flags;			///< Flags for loading
	char*						description;	///< Description, from which data must be loaded
	EVDS_OBJECT*				firstObject;	///< First loaded object (use OnLoadObject to get pointers for all objects)
	void*						userdata;		///< User-defined pointer
};


////////////////////////////////////////////////////////////////////////////////
/// @ingroup EVDS_OBJECT
/// @brief Information structure for EVDS_Object_SaveEx().
///
/// If filename is null, EVDS_OBJECT_SAVEEX::description will be set to pointer to
/// saved description.
///
/// @note If the object is saved as a description string, the string must be then free'd
///       by the application.
///
/// Additional flags can be specified to change default behavior of the saver:
/// Name									| Description
/// ----------------------------------------|----------------------
/// @c EVDS_OBJECT_SAVEEX_ONLY_CHILDREN		| Save only children of the object passed into EVDS_Object_SaveEx()
/// @c EVDS_OBJECT_SAVEEX_SAVE_UIDS			| Save unique ID's of objects
/// @c EVDS_OBJECT_SAVEEX_SAVE_FULL_STATE	| Save full state vector for object (including time and quaternion, solver states)
///
///	See EVDS_Object_SaveEx() for more information.
////////////////////////////////////////////////////////////////////////////////
#ifndef DOXYGEN_INTERNAL_STRUCTS
struct EVDS_OBJECT_SAVEEX_TAG {
#else
struct EVDS_OBJECT_SAVEEX {
#endif
	//EVDS_Callback_SaveObject*	OnSaveObject;	///< Called before every object is saved
	int							flags;			///< Flags for saving
	char*						description;	///< If filename is null, pointer to string will be written here
	void*						userdata;		///< User-defined pointer
};




////////////////////////////////////////////////////////////////////////////////
/// @ingroup EVDS_MESH
/// @{

/// Index to vertex, normal, or texture coordinates for 3D mesh.
typedef unsigned int EVDS_MESH_INDEX;

/// Do not generate triangles
#define EVDS_MESH_SKIP_TRIANGLES				1
/// Do not generate per-vertex information (normals, vertex info, list of vertices)
#define EVDS_MESH_SKIP_VERTICES					2
/// Do not generate per-vertex normals
#define EVDS_MESH_SKIP_VERTEX_NORMALS			4
/// Do not generate per-vertex information
#define EVDS_MESH_SKIP_VERTEX_INFO				8
/// Do not generate indices
#define EVDS_MESH_SKIP_INDICES					16
/// Do not generate edge data
#define EVDS_MESH_SKIP_EDGES					32
/// Do not generate additional triangles for thickness
#define EVDS_MESH_NO_THICKNESS					64
/// Force number of segments when generating mesh by cross-sections
#define EVDS_MESH_FORCE_NUMSEGMENTS				128
/// Use number of divisions instead of resolution as a quality parameter
#define EVDS_MESH_USE_DIVISIONS					256

/// Lowest resolution for the mesh
#define EVDS_MESH_LOWEST_RESOLUTION				(1.0f/EVDS_EPSf)

/// @}
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
/// @ingroup EVDS_MESH
/// @brief Information structure for EVDS_Mesh_GenerateEx().
///
/// If FIXME flag is specified, EVDS_MESH_GENERATEEX::target_index can be used to generate only
/// part of the mesh. The index specifies number of the cross-section defined volume to generate.
///
/// If @c EVDS_MESH_FORCE_NUMSEGMENTS flag is specified, EVDS_MESH_GENERATEEX::num_segments defines the target
/// number of segments that must be used for tessellation.
///
/// It's possible to limit resolution when @c EVDS_MESH_USE_DIVISIONS is used by setting EVDS_MESH_GENERATEEX::min_resolution.
///
/// Additional flags can be specified for generating the mesh:
/// Name								| Description
/// ------------------------------------|------------------------------
/// EVDS_MESH_SKIP_TRIANGLES			| Do not generate triangles
/// EVDS_MESH_SKIP_VERTICES				| Do not generate per-vertex information (normals, vertex info, list of vertices)
/// EVDS_MESH_SKIP_VERTEX_NORMALS		| Do not generate per-vertex normals
/// EVDS_MESH_SKIP_VERTEX_INFO			| Do not generate per-vertex information
/// EVDS_MESH_SKIP_INDICES				| Do not generate indices
/// EVDS_MESH_SKIP_EDGES				| Do not generate edge data
/// EVDS_MESH_NO_THICKNESS				| Do not generate additional triangles for thickness
/// EVDS_MESH_FORCE_NUMSEGMENTS			| Force number of segments when generating mesh by cross-sections
/// EVDS_MESH_USE_DIVISIONS				| Use number of divisions instead of resolution as a quality parameter
///
///	See EVDS_Mesh_GenerateEx() for more information.
////////////////////////////////////////////////////////////////////////////////
#ifndef DOXYGEN_INTERNAL_STRUCTS
struct EVDS_MESH_GENERATEEX_TAG {
#else
struct EVDS_MESH_GENERATEEX {
#endif
	float resolution;			///< Mesh resolution
	float min_resolution;		///< Smallest allowed resolution
	int flags;					///< Flags passed into EVDS_Mesh_Generate()
	int target_index;			///< Index of element for which mesh must be generated
	int num_segments;			///< Number of segments for tesselation (see EVDS_MESH_FORCE_NUMSEGMENTS)
};


////////////////////////////////////////////////////////////////////////////////
/// @ingroup EVDS_MESH
/// @brief Vector for 3D mesh.
///
/// Represents a single-precision vector for use with EVDS tesselator (see
/// EVDS_Mesh_GenerateEx() for more information).
////////////////////////////////////////////////////////////////////////////////
typedef struct EVDS_MESH_VECTOR_TAG {
	float x;				///< X component of the vector
	float y;				///< Y component of the vector
	float z;				///< Z component of the vector
} EVDS_MESH_VECTOR;


////////////////////////////////////////////////////////////////////////////////
/// @ingroup EVDS_MESH
/// @brief Single triangle of a 3D mesh.
///
/// EVDS_MESH_TRIANGLE::center defines the triangles center of mass, it is calculated
/// as an average of triangles endpoints.
///
/// EVDS_MESH_TRIANGLE::triangle_normal is the unit normal to triangles surface - it's
/// calculated from three endpoints and only represents normal for this triangle.
///
/// EVDS_MESH_TRIANGLE::vertex and EVDS_MESH_TRIANGLE::normal arrays contain vertex position
/// and normal value for every triangles endpoint. If triangle belongs to a smooth surface, the normals
/// will correspond to local normals to that smooth surface in the triangle endpoints.
///
/// EVDS_MESH_TRIANGLE::indices are the three indices of triangles endpoints in the meshes array of vertices.
///
/// EVDS_MESH_TRIANGLE::area is the total area of the triangle (always positive), EVDS_MESH_TRIANGLE::thickness
/// is the thickness of mesh under the triangle. If thickness is zero, then this triangle encloses some volume.
///
/// EVDS_MESH_TRIANGLE::cross_section is the index of cross-section to which this triangle belongs. It can be
/// used to segment cross-sections of a mesh (for visual or other purposes).
///
/// EVDS_MESH_TRIANGLE::smoothing_group is the index of smoothing group this triangle belongs to. Triangles within
/// same smoothing group make up a single smooth surface.
////////////////////////////////////////////////////////////////////////////////
typedef struct EVDS_MESH_TRIANGLE_TAG {
	EVDS_MESH_VECTOR center;				///< Location of triangles center of mass
	EVDS_MESH_VECTOR triangle_normal;		///< Normal to the triangle
	EVDS_MESH_VECTOR vertex[3];				///< Coordinates of triangle vertices
	EVDS_MESH_VECTOR normal[3];				///< Normal in every vertex (accounts for smooth and flat surfaces)
	EVDS_MESH_INDEX indices[3];				///< Indices of triangle vertices
	//EVDS_MESH_VECTOR smooth_normal[3];	///< Average normal in every vertex
	//struct EVDS_MESH_TRIANGLE_TAG* edge[3];	///< Triangle at edge (can be null)
	float area;								///< Triangle area
	float thickness;						///< Triangle thickness (if zero, then triangle encloses a volume)
	int cross_section;						///< Index of cross-section this triangle belongs to
	int smoothing_group;					///< Index of the smoothing group this triangle belongs to
} EVDS_MESH_TRIANGLE;


////////////////////////////////////////////////////////////////////////////////
/// @ingroup EVDS_MESH
/// @brief Additional information about a single vertex.
///
/// EVDS_MESH_VERTEX_INFO::area defines area per single vertex - total sum of areas of all
/// vertices will be equal to total surface area of the mesh. Each vertices area is a weighted sum of
/// areas of triangles which include this vertex.
////////////////////////////////////////////////////////////////////////////////
typedef struct EVDS_MESH_VERTEX_INFO_TAG {
	EVDS_MESH_INDEX* triangles;				///< Triangles that this vertex belongs to
	EVDS_MESH_INDEX* tri_index;				///< Index of vertex in the triangle
	float area;								///< Total area of this vertex
	int cross_section;						///< Index of cross-section this vertex belongs to
	int smoothing_group;					///< Index of the smoothing group this vertex belongs to
	int num_triangles;						///< Number of triangles that share this vertex
	int num_allocated;						///< Size of array of triangles (used internally)
} EVDS_MESH_VERTEX_INFO;


////////////////////////////////////////////////////////////////////////////////
/// @ingroup EVDS_MESH
/// @brief 3D procedural mesh of an object.
///
/// A 3D mesh may be generated for every object (as long as the object geometry data can be recognized
/// by the built-in tessellator). This mesh can be used for physics or rendering afterwards.
///
/// Additionally approximate information about total volume and area will be returned for the
/// given resolution level, as well as the bounding box for the mesh.
///
/// See EVDS_MESH_VERTEX_INFO, EVDS_MESH_VECTOR, EVDS_MESH_TRIANGLE for more information on data
/// structures that make up the EVDS_MESH.
///
/// @note Not all data will be present in the data structure, depending on flags passed to
///       EVDS_Mesh_Generate(). Using data structures which were not requested via flags will put
///       program into undefined state (most likely result in a crash).
////////////////////////////////////////////////////////////////////////////////
typedef struct EVDS_MESH_TAG {
	EVDS_OBJECT* object;					///< Object that this mesh represents
	EVDS_MESH_INTERNAL* internal;			///< Internal data used by EVDS_Mesh_GenerateEx()
	EVDS_MESH_TRIANGLE* triangles;			///< Array of triangles
	EVDS_MESH_INDEX* indices;				///< Array of indices
	EVDS_MESH_VECTOR* normals;				///< Array of normals
	EVDS_MESH_VECTOR* vertices;				///< Array of vertices
	EVDS_MESH_VERTEX_INFO* vertex_info;		///< Array of additional information about each vertex
	int num_triangles;						///< Total number of triangles
	int num_indices;						///< Total number of indices
	int num_vertices;						///< Total number of vertices
	int num_smoothing_groups;				///< Total number of smoothing groups
	float total_volume;						///< Total volume of the mesh
	float total_area;						///< Total area of the mesh
	EVDS_MESH_VECTOR bbox_min;				///< Minimum set of coordinates for bounding box
	EVDS_MESH_VECTOR bbox_max;				///< Maximum set of coordinates for bounding box
} EVDS_MESH;




////////////////////////////////////////////////////////////////////////////////
/// @ingroup EVDS_ENVIRONMENT
/// @{

/// Helium
#define EVDS_ENVIRONMENT_SPECIES_HE		0
/// Atomic oxygen
#define EVDS_ENVIRONMENT_SPECIES_O		1
/// Molecular oxygen
#define EVDS_ENVIRONMENT_SPECIES_O2		2
/// Atomic nitrogen
#define EVDS_ENVIRONMENT_SPECIES_N		3
/// Molecular nitrogen
#define EVDS_ENVIRONMENT_SPECIES_N2		4
/// Argon
#define EVDS_ENVIRONMENT_SPECIES_AR		5
/// Atomic hydrogen
#define EVDS_ENVIRONMENT_SPECIES_H		6
/// Carbon dioxide
#define EVDS_ENVIRONMENT_SPECIES_CO2	7
/// Water vapour
#define EVDS_ENVIRONMENT_SPECIES_H2O	7
/// Nitrogen oxide
#define EVDS_ENVIRONMENT_SPECIES_NO		8
/// Neon
#define EVDS_ENVIRONMENT_SPECIES_Ne		9
/// Krypton
#define EVDS_ENVIRONMENT_SPECIES_Kr		10
/// Ozone
#define EVDS_ENVIRONMENT_SPECIES_O3		11
/// Methane
#define EVDS_ENVIRONMENT_SPECIES_CH4	12
/// Sulfur dioxide
#define EVDS_ENVIRONMENT_SPECIES_SO2	13
/// Other species which are not listed above
#define EVDS_ENVIRONMENT_SPECIES_OTHER	14
/// Maximum species count
#define EVDS_ENVIRONMENT_SPECIES_MAX	15

/// @}
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
/// @ingroup EVDS_ENVIRONMENT
/// @brief Structure that describes state of the atmosphere.
///
/// The structure is filled out by the atmospheric model. Missing/invalid parameters
/// will be automatically filled out by the EVDS simulator using naive guesses for unknown
/// parameters.
////////////////////////////////////////////////////////////////////////////////
typedef struct EVDS_ENVIRONMENT_ATMOSPHERE_TAG {
	EVDS_REAL density;						///< Atmospheric density [kg/m3]
	EVDS_REAL pressure;						///< Atmospheric pressure [Pa]
	EVDS_REAL temperature;					///< Atmospheric temperature [K]
	EVDS_REAL concentration;				///< Total atmospheric concentration [1/m3]

	EVDS_REAL partial_density[EVDS_ENVIRONMENT_SPECIES_MAX]; ///< Partial density of each species [kg/m3]
	EVDS_REAL partial_concentration[EVDS_ENVIRONMENT_SPECIES_MAX]; ///< Partial concentration of each species [1/m3]
} EVDS_ENVIRONMENT_ATMOSPHERE;


////////////////////////////////////////////////////////////////////////////////
/// @ingroup EVDS_ENVIRONMENT
/// @brief Structure describing the radiation environment (placeholder).
////////////////////////////////////////////////////////////////////////////////
typedef struct EVDS_ENVIRONMENT_RADIATION_TAG {
	void* userdata;							///< Pointer to user data 
} EVDS_ENVIRONMENT_RADIATION;


////////////////////////////////////////////////////////////////////////////////
/// @ingroup EVDS_ENVIRONMENT
/// @brief Structure that describes a single sound
///
/// @todo
////////////////////////////////////////////////////////////////////////////////
typedef struct EVDS_SOUND_TAG {
	int id;						/// ID of the sound (defines which actual sound must be played)
	EVDS_OBJECT* source;		/// Source object which generated this sound

	EVDS_REAL time;				/// Time at which sound started
	EVDS_REAL duration;			/// Duration of the sound (<0.0 means indefinite)
	EVDS_VECTOR position;		/// Location in space
	EVDS_REAL parameter[4];		/// Arbitrary parameters
} EVDS_SOUND;




////////////////////////////////////////////////////////////////////////////////
/// @ingroup EVDS_ENVIRONMENT
/// @{

/// Callback for computing gravitational field
typedef int EVDS_Callback_GetGravitationalField(EVDS_OBJECT* object, EVDS_VECTOR* r, EVDS_REAL* phi, EVDS_VECTOR* field);
/// Callback for computing gravity gradient torque
typedef int EVDS_Callback_GetGravityGradientTorque(EVDS_OBJECT* object, EVDS_VECTOR* r, EVDS_VECTOR* torque);
/// Callback for computing magnetic field
typedef int EVDS_Callback_GetMagneticField(EVDS_OBJECT* object, EVDS_VECTOR* r, EVDS_VECTOR* field);
/// Callback for computing local atmosphere of the planet
typedef int EVDS_Callback_GetAtmosphericData(EVDS_OBJECT* object, EVDS_VECTOR* r, EVDS_ENVIRONMENT_ATMOSPHERE* atmosphere);
/// Callback for computing radiation environment
typedef int EVDS_Callback_GetRadiationData(EVDS_OBJECT* object, EVDS_VECTOR* r, EVDS_ENVIRONMENT_RADIATION* radiation);

/// @}
////////////////////////////////////////////////////////////////////////////////




// Include data structures themselves
#ifdef EVDS_LIBRARY
#	include "evds_internal.h"
#endif





////////////////////////////////////////////////////////////////////////////////
// Variable types
////////////////////////////////////////////////////////////////////////////////
/// @ingroup EVDS_VARIABLE
/// @{

/// Floating point value
#define EVDS_VARIABLE_TYPE_FLOAT		0
/// Null-terminated string
#define EVDS_VARIABLE_TYPE_STRING		1
/// Vector (3-dimensional)
#define EVDS_VARIABLE_TYPE_VECTOR		2
/// Quaternion
#define EVDS_VARIABLE_TYPE_QUATERNION	3
/// Data structure (contains a list of nested variables, list of attributes)
#define EVDS_VARIABLE_TYPE_NESTED		4
/// Stores pointer to custom data (some C structure)
#define EVDS_VARIABLE_TYPE_DATA_PTR		5
/// Stores a C function/callback pointer. Function signature depends on variable name
#define EVDS_VARIABLE_TYPE_FUNCTION_PTR	6
/// Multi-dimensional interpolated function defined by polynomials or a table
#define EVDS_VARIABLE_TYPE_FUNCTION		7

/// @}
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// Derivative levels/vector types
////////////////////////////////////////////////////////////////////////////////
/// @ingroup EVDS_MATH
/// @{

/// Direction vector (units: \f$dimensionless\f$)
#define EVDS_VECTOR_DIRECTION				10
/// Position displacement vector (units: \f$m\f$)
#define EVDS_VECTOR_DISPLACEMENT			10
/// Force vector (units: \f$N\f$)
#define EVDS_VECTOR_FORCE					10
/// Torque vector (units: \f$N \cdot m\f$)
#define EVDS_VECTOR_TORQUE					11
/// Position vector (units: \f$m\f$)
#define EVDS_VECTOR_POSITION				0
/// Velocity vector (units: \f$\frac{m}{sec}\f$)
#define EVDS_VECTOR_VELOCITY				1
/// Acceleration vector (units: \f$\frac{m}{sec^2}\f$)
#define EVDS_VECTOR_ACCELERATION			2
/// Angular Velocity vector (units: \f$\frac{rad}{sec}\f$)
#define EVDS_VECTOR_ANGULAR_VELOCITY		-1
/// Angular acceleration vector (units: \f$\frac{rad}{sec^2}\f$)
#define EVDS_VECTOR_ANGULAR_ACCELERATION	-2
/// Vector that must be transformed without non-inertial components
#define EVDS_VECTOR_INERTIAL_TRANSFORM		12

/// @}
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// Error codes
////////////////////////////////////////////////////////////////////////////////
/// @ingroup EVDS_BASIC
/// @{

/// Completed successfully
#define EVDS_OK								0
/// Internal error (used for errors in internal APIs)
#define EVDS_ERROR_INTERNAL					1
/// Error opening file (file not found or not accessible)
#define EVDS_ERROR_FILE						2
/// Syntax error in configuration string/file
#define EVDS_ERROR_SYNTAX					3
/// Error in memory allocation
#define EVDS_ERROR_MEMORY					4
/// Parameter invalid or out of range
#define EVDS_ERROR_BAD_PARAMETER			5
/// This function is not compatible with the current state
#define EVDS_ERROR_BAD_STATE				6
/// API call is not in the right thread (some calls must be within same thread)
#define EVDS_ERROR_INTERTHREAD_CALL			7
/// Object was removed, or the data is otherwise no longer valid
#define EVDS_ERROR_INVALID_OBJECT			8
/// Object/variable not found
#define EVDS_ERROR_NOT_FOUND				9
/// Object not yet initialized
#define EVDS_ERROR_NOT_INITIALIZED			10
/// API call for given set of parameters is not implemented
#define EVDS_ERROR_NOT_IMPLEMENTED			11
/// Invalid object type (see EVDS_Object_CheckType())
#define EVDS_ERROR_INVALID_TYPE				12


/// Object is ignored by solver
#define EVDS_IGNORE_OBJECT					0
/// Object is claimed by solver
#define EVDS_CLAIM_OBJECT					13

/// @}
////////////////////////////////////////////////////////////////////////////////








////////////////////////////////////////////////////////////////////////////////
/// @defgroup EVDS_SYSTEM System API
/// @brief API to initialize and work with EVDS_SYSTEM object.
///
/// @{
////////////////////////////////////////////////////////////////////////////////
// Create EVDS system
EVDS_API int EVDS_System_Create(EVDS_SYSTEM** p_system);
// Destroy EVDS system and all resources. All threads must be shutdown first!
EVDS_API int EVDS_System_Destroy(EVDS_SYSTEM* system);

// Set EVDS system global time (MJD)
EVDS_API int EVDS_System_SetTime(EVDS_SYSTEM* system, EVDS_REAL time);
// Get EVDS system global time (MJD)
EVDS_API int EVDS_System_GetTime(EVDS_SYSTEM* system, EVDS_REAL* time);

// Get root inertial space object
EVDS_API int EVDS_System_GetRootInertialSpace(EVDS_SYSTEM* system, EVDS_OBJECT** p_object);
// Get objects by type
EVDS_API int EVDS_System_GetObjectsByType(EVDS_SYSTEM* system, const char* type, SIMC_LIST** p_list);
// Get object by UID (can search in children of a given object)
EVDS_API int EVDS_System_GetObjectByUID(EVDS_SYSTEM* system, EVDS_OBJECT* parent, unsigned int uid, EVDS_OBJECT** p_object);
// Get object by name (can search in children of a given object)
EVDS_API int EVDS_System_GetObjectByName(EVDS_SYSTEM* system, EVDS_OBJECT* parent, const char* name, EVDS_OBJECT** p_object);
// Query a variable/object by data reference
EVDS_API int EVDS_System_QueryByReference(EVDS_SYSTEM* system, EVDS_OBJECT* parent, const char* query, EVDS_VARIABLE** p_variable, EVDS_OBJECT** p_object);

// Cleanup objects (mandatory to call once in a while, in multithreaded environment only)
EVDS_API int EVDS_System_CleanupObjects(EVDS_SYSTEM* system);

// Load database from a file
EVDS_API int EVDS_System_DatabaseFromFile(EVDS_SYSTEM* system, const char* filename);
// Load database from a string
EVDS_API int EVDS_System_DatabaseFromString(EVDS_SYSTEM* system, const char* description);
// Get database by name
EVDS_API int EVDS_System_GetDatabaseByName(EVDS_SYSTEM* system, const char* name, EVDS_VARIABLE** p_database);
// Get list of all databases
EVDS_API int EVDS_System_GetDatabasesList(EVDS_SYSTEM* system, SIMC_LIST** p_list);
// Get list of all entries in database
EVDS_API int EVDS_System_GetDatabaseEntries(EVDS_SYSTEM* system, const char* name, SIMC_LIST** p_list);

// Set global callbacks
EVDS_API int EVDS_System_SetGlobalCallbacks(EVDS_SYSTEM* system, EVDS_GLOBAL_CALLBACKS* p_callbacks);

// Set userdata
EVDS_API int EVDS_System_SetUserdata(EVDS_SYSTEM* system, void* userdata);
// Get userdata
EVDS_API int EVDS_System_GetUserdata(EVDS_SYSTEM* system, void** p_userdata);
////////////////////////////////////////////////////////////////////////////////
/// @}
////////////////////////////////////////////////////////////////////////////////








////////////////////////////////////////////////////////////////////////////////
/// @defgroup EVDS_UTIL Miscellaneous API
/// @brief Logging and miscellanous functions
///
/// @{
////////////////////////////////////////////////////////////////////////////////
// Get the library version
EVDS_API int EVDS_Version(int* version, char* version_string);
// Set global logging callback
EVDS_API int EVDS_SetLogCallback(EVDS_Callback_Log* callback);
// Convert a null-terminated string to an EVDS_REAL (parses units from input string, value is in metric units)
EVDS_API int EVDS_StringToReal(const char* str, char** str_end, EVDS_REAL* p_value);
////////////////////////////////////////////////////////////////////////////////
/// @}
////////////////////////////////////////////////////////////////////////////////








////////////////////////////////////////////////////////////////////////////////
/// @defgroup EVDS_SOLVER Solver API
/// @brief API to register EVDS_SOLVER object within EVDS_SYSTEM
///
/// @{
////////////////////////////////////////////////////////////////////////////////
// Registers a new solver
EVDS_API int EVDS_Solver_Register(EVDS_SYSTEM* system, EVDS_SOLVER* solver);
////////////////////////////////////////////////////////////////////////////////
/// @}
////////////////////////////////////////////////////////////////////////////////








////////////////////////////////////////////////////////////////////////////////
/// @defgroup EVDS_OBJECT Object API
/// @brief Implements API to work with objects (EVDS_OBJECT)
///
/// EVDS_OBJECT represents a single "unit" in EVDS simulator. It's defined by
/// its type name and set of variables inside it. See data structure documentation
/// for more information.
///
/// @{
////////////////////////////////////////////////////////////////////////////////

// Create new object
EVDS_API int EVDS_Object_Create(EVDS_OBJECT* parent, EVDS_OBJECT** p_object);
// Create new object by another object or return already existing object (used for objects created by solvers)
EVDS_API int EVDS_Object_CreateBy(EVDS_OBJECT* origin, const char* sub_name, EVDS_OBJECT* parent, EVDS_OBJECT** p_object);
// Load object variables from a file. Will only return first pointer of all loaded vessels (other vessels will be initialized)
EVDS_API int EVDS_Object_LoadFromFile(EVDS_OBJECT* parent, const char* filename, EVDS_OBJECT** p_object);
// Load object variables from a configuration string. Same behaviour as previous function
EVDS_API int EVDS_Object_LoadFromString(EVDS_OBJECT* parent, const char* description, EVDS_OBJECT** p_object);
// Load object. Extra information specified by info structure
EVDS_API int EVDS_Object_LoadEx(EVDS_OBJECT* parent, const char* filename, EVDS_OBJECT_LOADEX* info);
// Create object as a copy of a different object
EVDS_API int EVDS_Object_Copy(EVDS_OBJECT* source, EVDS_OBJECT* parent, EVDS_OBJECT** p_object);
// Create object as a copy of a different object (do not copy objects children)
EVDS_API int EVDS_Object_CopySingle(EVDS_OBJECT* source, EVDS_OBJECT* parent, EVDS_OBJECT** p_object);
// Create copy of objects children under another parent (only copies objects children)
EVDS_API int EVDS_Object_CopyChildren(EVDS_OBJECT* source_parent, EVDS_OBJECT* parent);
// Move all objects children to another parent (only moves objects children)
EVDS_API int EVDS_Object_MoveChildren(EVDS_OBJECT* source_parent, EVDS_OBJECT* parent);
// Save object and its children to file
EVDS_API int EVDS_Object_SaveToFile(EVDS_OBJECT* object, const char* filename);
// Save object and return its description
EVDS_API int EVDS_Object_SaveToString(EVDS_OBJECT* object, char** description);
// Save object. Extra information specified by info structure
EVDS_API int EVDS_Object_SaveEx(EVDS_OBJECT* object, const char* filename, EVDS_OBJECT_SAVEEX* info);
// Destroy object (the data may remain in memory until object is free'd in other threads too)
EVDS_API int EVDS_Object_Destroy(EVDS_OBJECT* object);

// Initialize object and start working with it
EVDS_API int EVDS_Object_Initialize(EVDS_OBJECT* object, int is_blocking);
// Check if object is initialized
EVDS_API int EVDS_Object_IsInitialized(EVDS_OBJECT* object, int* is_initialized);
// Transfer initialization ownership to current thread (current thread becomes responsible for initializing object)
EVDS_API int EVDS_Object_TransferInitialization(EVDS_OBJECT* object);

// Signal that object is stored somewhere
EVDS_API int EVDS_Object_Store(EVDS_OBJECT* object);
// Signal that objects is no longer stored somewhere
EVDS_API int EVDS_Object_Release(EVDS_OBJECT* object);
// Is the object destroyed (the data will not be cleared to avoid any exceptions)
EVDS_API int EVDS_Object_IsDestroyed(EVDS_OBJECT* object, int* is_destroyed);

// Solve object
EVDS_API int EVDS_Object_Solve(EVDS_OBJECT* object, EVDS_REAL delta_time);
// Integrate object. Sets object state vector to "state" and returns "derivative" at that point (does not reset state!)
//  Pass "0" as state to find derivative at current state
EVDS_API int EVDS_Object_Integrate(EVDS_OBJECT* object, EVDS_REAL delta_time, EVDS_STATE_VECTOR* state,
								   EVDS_STATE_VECTOR_DERIVATIVE* derivative);

// Set objects solver
EVDS_API int EVDS_Object_SetCallback_OnSolve(EVDS_OBJECT* object, EVDS_Callback_Solve* p_callback);
// Set objects integrator
EVDS_API int EVDS_Object_SetCallback_OnIntegrate(EVDS_OBJECT* object, EVDS_Callback_Integrate* p_callback);

// Set object type
EVDS_API int EVDS_Object_SetType(EVDS_OBJECT* object, const char* type);
// Set object name
EVDS_API int EVDS_Object_SetName(EVDS_OBJECT* object, const char* name);
// Updates the objects name to be unique within its parent (or target parent)
EVDS_API int EVDS_Object_SetUniqueName(EVDS_OBJECT* object, EVDS_OBJECT* parent);
// Add a new variable
EVDS_API int EVDS_Object_AddVariable(EVDS_OBJECT* object, const char* name, EVDS_VARIABLE_TYPE type, EVDS_VARIABLE** p_variable);
// Add a new variable (floating point value, can accept 0 as p_variable)
EVDS_API int EVDS_Object_AddRealVariable(EVDS_OBJECT* object, const char* name, EVDS_REAL value, EVDS_VARIABLE** p_variable);

// Check object type (only after initialized OR only in initializers thread)
EVDS_API int EVDS_Object_CheckType(EVDS_OBJECT* object, const char* type);
// Get object type (only after initialized OR only in initializers thread)
EVDS_API int EVDS_Object_GetType(EVDS_OBJECT* object, char* type, size_t max_length);
// Get object name (only after initialized OR only in initializers thread)
EVDS_API int EVDS_Object_GetName(EVDS_OBJECT* object, char* name, size_t max_length);
// Get variable by name (only after initialized OR only in initializers thread)
EVDS_API int EVDS_Object_GetVariable(EVDS_OBJECT* object, const char* name, EVDS_VARIABLE** p_variable);
// Get all variables (only after initialized OR only in initializers thread)
EVDS_API int EVDS_Object_GetVariables(EVDS_OBJECT* object, SIMC_LIST** p_list);
// Get floating-point variable by name (only after initialized OR only in initializers thread)
EVDS_API int EVDS_Object_GetRealVariable(EVDS_OBJECT* object, const char* name, EVDS_REAL* value, EVDS_VARIABLE** p_variable);
// Get objects reference in the system (for use with EVDS_System_QueryByReference())
EVDS_API int EVDS_Object_GetReference(EVDS_OBJECT* object, EVDS_OBJECT* root, char* reference, size_t max_length);

// Get children
EVDS_API int EVDS_Object_GetChildren(EVDS_OBJECT* object, SIMC_LIST** p_list);
// Get all children (including uninitialized)
EVDS_API int EVDS_Object_GetAllChildren(EVDS_OBJECT* object, SIMC_LIST** p_list);
// Get parent
EVDS_API int EVDS_Object_GetParent(EVDS_OBJECT* object, EVDS_OBJECT** p_object);
// Get first parent object by given type (can contain wildcards)
EVDS_API int EVDS_Object_GetParentObjectByType(EVDS_OBJECT* object, const char* type, EVDS_OBJECT** p_object);

// Change parent for the object
EVDS_API int EVDS_Object_SetParent(EVDS_OBJECT* object, EVDS_OBJECT* new_parent);
// Place object in front of "head" in list of children objects
EVDS_API int EVDS_Object_MoveInList(EVDS_OBJECT* object, EVDS_OBJECT* head);

// Set unique ID
EVDS_API int EVDS_Object_SetUID(EVDS_OBJECT* object, unsigned int uid);
// Get unique ID
EVDS_API int EVDS_Object_GetUID(EVDS_OBJECT* object, unsigned int* uid);

// Get current state vector and its derivative
EVDS_API int EVDS_Object_GetStateVector(EVDS_OBJECT* object, EVDS_STATE_VECTOR* vector);
// Set state vector and derivative
EVDS_API int EVDS_Object_SetStateVector(EVDS_OBJECT* object, EVDS_STATE_VECTOR* vector);

// Get previous state vector (can be used for interpolation)
EVDS_API int EVDS_Object_GetPreviousStateVector(EVDS_OBJECT* object, EVDS_STATE_VECTOR* vector);
// Get state vector interpolated between current and previous one
EVDS_API int EVDS_Object_GetInterpolatedStateVector(EVDS_OBJECT* object, EVDS_STATE_VECTOR* vector, double t);

//FIXME: future/unsupported API
// Start rendering object (pass interpolated state vector or any preferred state vector to render from)
//EVDS_API int EVDS_Object_StartRendering(EVDS_OBJECT* object, EVDS_STATE_VECTOR* vector);
// End drawing object
//EVDS_API int EVDS_Object_EndRendering(EVDS_OBJECT* object);

// Shortcut to get center of mass position
EVDS_API int EVDS_Object_GetCoMPosition(EVDS_OBJECT* object, EVDS_VECTOR* p_vector);

// Shortcut to set position
EVDS_API int EVDS_Object_SetPosition(EVDS_OBJECT* object, EVDS_OBJECT* target_coordinates, EVDS_REAL x, EVDS_REAL y, EVDS_REAL z);
// Shortcut to set velocity
EVDS_API int EVDS_Object_SetVelocity(EVDS_OBJECT* object, EVDS_OBJECT* target_coordinates, EVDS_REAL vx, EVDS_REAL vy, EVDS_REAL vz);
// Shortcut to set angular velocity (roll-pitch-yaw order)
EVDS_API int EVDS_Object_SetAngularVelocity(EVDS_OBJECT* object, EVDS_OBJECT* target_coordinates, EVDS_REAL r, EVDS_REAL p, EVDS_REAL q);
// Shortcut to set orientation (in euler angles)
EVDS_API int EVDS_Object_SetOrientation(EVDS_OBJECT* object, EVDS_OBJECT* target_coordinates, EVDS_REAL roll, EVDS_REAL pitch, EVDS_REAL yaw);
// Shortcut to set orientation (as a quaternion)
EVDS_API int EVDS_Object_SetOrientationQuaternion(EVDS_OBJECT* object, EVDS_QUATERNION* q);

// Shortcut to set position of center of mass
EVDS_API int EVDS_Object_SetCoMPosition(EVDS_OBJECT* object, EVDS_OBJECT* target_coordinates, EVDS_REAL x, EVDS_REAL y, EVDS_REAL z);
// Shortcut to set velocity of center of mass
EVDS_API int EVDS_Object_SetCoMVelocity(EVDS_OBJECT* object, EVDS_OBJECT* target_coordinates, EVDS_REAL vx, EVDS_REAL vy, EVDS_REAL vz);
// Shortcut to set orientation around center of mass (in euler angles)
EVDS_API int EVDS_Object_SetCoMOrientation(EVDS_OBJECT* object, EVDS_OBJECT* target_coordinates, EVDS_REAL roll, EVDS_REAL pitch, EVDS_REAL yaw);
// Shortcut to set orientation around center of mass (as a quaternion)
EVDS_API int EVDS_Object_SetCoMOrientationQuaternion(EVDS_OBJECT* object, EVDS_QUATERNION* q);

// Set time for which object position is valid
EVDS_API int EVDS_Object_SetStateTime(EVDS_OBJECT* object, EVDS_REAL t);

// Set userdata
EVDS_API int EVDS_Object_SetUserdata(EVDS_OBJECT* object, void* userdata);
// Get userdata
EVDS_API int EVDS_Object_GetUserdata(EVDS_OBJECT* object, void** p_userdata);

// Set solverdata
EVDS_API int EVDS_Object_SetSolverdata(EVDS_OBJECT* object, void* solverdata);
// Get solverdata
EVDS_API int EVDS_Object_GetSolverdata(EVDS_OBJECT* object, void** solverdata);

// Get system from object
EVDS_API int EVDS_Object_GetSystem(EVDS_OBJECT* object, EVDS_SYSTEM** p_system);
////////////////////////////////////////////////////////////////////////////////
/// @}
////////////////////////////////////////////////////////////////////////////////








////////////////////////////////////////////////////////////////////////////////
/// @defgroup EVDS_VARIABLE Variable API
/// @brief Implements API to work with variables (EVDS_VARIABLE)
///
/// @{
////////////////////////////////////////////////////////////////////////////////
// Add a nested variable
EVDS_API int EVDS_Variable_AddNested(EVDS_VARIABLE* parent_variable, const char* name, EVDS_VARIABLE_TYPE type, EVDS_VARIABLE** p_variable);
// Add an attribute to a variable
EVDS_API int EVDS_Variable_AddAttribute(EVDS_VARIABLE* parent_variable, const char* name, EVDS_VARIABLE_TYPE type, EVDS_VARIABLE** p_variable);
// Add a floating-point attribute to a variable
EVDS_API int EVDS_Variable_AddFloatAttribute(EVDS_VARIABLE* parent_variable, const char* name, EVDS_REAL value, EVDS_VARIABLE** p_variable);
// Get variables attribute by name
EVDS_API int EVDS_Variable_GetAttribute(EVDS_VARIABLE* parent_variable, const char* name, EVDS_VARIABLE** p_variable);
// Get variables nested variable by name
EVDS_API int EVDS_Variable_GetNested(EVDS_VARIABLE* parent_variable, const char* name, EVDS_VARIABLE** p_variable);
// Delete a variable or an attribute
EVDS_API int EVDS_Variable_Destroy(EVDS_VARIABLE* variable);
// Move variable in parent list of nested variables
EVDS_API int EVDS_Variable_MoveInList(EVDS_VARIABLE* variable, EVDS_VARIABLE* head);

// Get variables name
EVDS_API int EVDS_Variable_GetName(EVDS_VARIABLE* variable, char* name, size_t max_length);
// Set variables name
EVDS_API int EVDS_Variable_SetName(EVDS_VARIABLE* variable, const char* name);
// Get variables type
EVDS_API int EVDS_Variable_GetType(EVDS_VARIABLE* variable, EVDS_VARIABLE_TYPE* type);

// Get list of nested variables (only after initialized OR only in initializers thread)
EVDS_API int EVDS_Variable_GetList(EVDS_VARIABLE* variable, SIMC_LIST** p_list);
// Get list of nested attributes (only after initialized OR only in initializers thread)
EVDS_API int EVDS_Variable_GetAttributes(EVDS_VARIABLE* variable, SIMC_LIST** p_list);
// Set real value
EVDS_API int EVDS_Variable_SetReal(EVDS_VARIABLE* variable, EVDS_REAL value);
// Get real value
EVDS_API int EVDS_Variable_GetReal(EVDS_VARIABLE* variable, EVDS_REAL* value);
// Set string value
EVDS_API int EVDS_Variable_SetString(EVDS_VARIABLE* variable, char* value, size_t length);
// Get string value
EVDS_API int EVDS_Variable_GetString(EVDS_VARIABLE* variable, char* value, size_t max_length, size_t* length);
// Get vector object (copies data into "value")
EVDS_API int EVDS_Variable_GetVector(EVDS_VARIABLE* variable, EVDS_VECTOR* value);
// Get quaternion object
EVDS_API int EVDS_Variable_GetQuaternion(EVDS_VARIABLE* variable, EVDS_QUATERNION* value);
// Set vector object
EVDS_API int EVDS_Variable_SetVector(EVDS_VARIABLE* variable, EVDS_VECTOR* value);
// Set quaternion object
EVDS_API int EVDS_Variable_SetQuaternion(EVDS_VARIABLE* variable, EVDS_QUATERNION* value);
// Set pointer to data structure
EVDS_API int EVDS_Variable_SetDataPointer(EVDS_VARIABLE* variable, void* data);
// Get pointer to data structure
EVDS_API int EVDS_Variable_GetDataPointer(EVDS_VARIABLE* variable, void** data);
// Set pointer to callback function
EVDS_API int EVDS_Variable_SetFunctionPointer(EVDS_VARIABLE* variable, void* data);
// Get pointer to callback function
EVDS_API int EVDS_Variable_GetFunctionPointer(EVDS_VARIABLE* variable, void** data);
// Get value from a 1D/2D/3D function 
EVDS_API int EVDS_Variable_GetFunctionValue(EVDS_VARIABLE* variable, EVDS_REAL x, EVDS_REAL y, EVDS_REAL z, EVDS_REAL* p_value);

// Convert variable to a printable string
EVDS_API int EVDS_Variable_ToString(EVDS_VARIABLE* variable, char* string, size_t max_length);
// Return printable string that represents variables value
EVDS_API char* EVDS_Variable_AsString(EVDS_VARIABLE* variable);

// Set userdata
EVDS_API int EVDS_Variable_SetUserdata(EVDS_VARIABLE* variable, void* userdata);
// Get userdata
EVDS_API int EVDS_Variable_GetUserdata(EVDS_VARIABLE* variable, void** p_userdata);
////////////////////////////////////////////////////////////////////////////////
/// @}
////////////////////////////////////////////////////////////////////////////////








////////////////////////////////////////////////////////////////////////////////
/// @defgroup EVDS_MATH Vector & Quaternion Math API
/// @brief Aerospace-oriented vection & quaternion math library
///
/// @{
////////////////////////////////////////////////////////////////////////////////
/// Macro to initialize a vector variable (must be called before this variable is used in any operations)
#define EVDS_Vector_Initialize(v) (	(v).x = 0, (v).y = 0, (v).z = 0, (v).derivative_level = 0, (v).coordinate_system = 0, (v).pcoordinate_system = 0, (v).vcoordinate_system = 0 )

// Convert to vector to target coordinate system
EVDS_API void EVDS_Vector_Convert(EVDS_VECTOR* target, EVDS_VECTOR* v, EVDS_OBJECT* target_coordinates);
// Convert quaternion to target coordinate system
EVDS_API void EVDS_Quaternion_Convert(EVDS_QUATERNION* target, EVDS_QUATERNION* q, EVDS_OBJECT* target_coordinates);

// Get raw numerical values for the vector inside target coordinate systems
EVDS_API void EVDS_Vector_Get(EVDS_VECTOR* v, EVDS_REAL* x, EVDS_REAL* y, EVDS_REAL* z, EVDS_OBJECT* target_coordinates);
// Shortcut to set vector values
EVDS_API void EVDS_Vector_Set(EVDS_VECTOR* target, int derivative_level, EVDS_OBJECT* target_coordinates, EVDS_REAL x, EVDS_REAL y, EVDS_REAL z);

// Shortcut to set vector position
EVDS_API void EVDS_Vector_SetPosition(EVDS_VECTOR* target, EVDS_OBJECT* target_coordinates, EVDS_REAL x, EVDS_REAL y, EVDS_REAL z);
// Shortcut to get vector from position
EVDS_API void EVDS_Vector_GetPositionVector(EVDS_VECTOR* v, EVDS_VECTOR* position);
// Shortcut to set position by vector
EVDS_API void EVDS_Vector_SetPositionVector(EVDS_VECTOR* v, EVDS_VECTOR* position);
// Shortcut to set vector velocity
EVDS_API void EVDS_Vector_SetVelocity(EVDS_VECTOR* target, EVDS_OBJECT* target_coordinates, EVDS_REAL x, EVDS_REAL y, EVDS_REAL z);
// Shortcut to get vector from velocity
EVDS_API void EVDS_Vector_GetVelocityVector(EVDS_VECTOR* v, EVDS_VECTOR* velocity);
// Shortcut to set velocity by vector
EVDS_API void EVDS_Vector_SetVelocityVector(EVDS_VECTOR* v, EVDS_VECTOR* velocity);

// Change position where force is applied
EVDS_API void EVDS_Vector_MoveForceToPosition(EVDS_VECTOR* force, EVDS_VECTOR* torque, EVDS_VECTOR* new_position);
// Change position where torque is applied
EVDS_API void EVDS_Vector_MoveTorqueToPosition(EVDS_VECTOR* torque, EVDS_VECTOR* new_position);
// Nullify the vector velocity and position
EVDS_API void EVDS_Vector_NullifyPositionAndVelocity(EVDS_VECTOR* v);

// Copy one vector into another
EVDS_API void EVDS_Vector_Copy(EVDS_VECTOR* target, EVDS_VECTOR* v);
// Copy one quaternion into another
EVDS_API void EVDS_Quaternion_Copy(EVDS_QUATERNION* target, EVDS_QUATERNION* q);
// Copy one state vector into another
EVDS_API void EVDS_StateVector_Copy(EVDS_STATE_VECTOR* target, EVDS_STATE_VECTOR* v);
// Copy one state vector into another
EVDS_API void EVDS_StateVector_Derivative_Copy(EVDS_STATE_VECTOR_DERIVATIVE* target, EVDS_STATE_VECTOR_DERIVATIVE* v);

// Add two vectors. Resulting vector is in coordinate system of v1
EVDS_API void EVDS_Vector_Add(EVDS_VECTOR* target, EVDS_VECTOR* v1, EVDS_VECTOR* v2);
// Subtract two vectors. Resulting vector is in coordinate system of v1
EVDS_API void EVDS_Vector_Subtract(EVDS_VECTOR* target, EVDS_VECTOR* v1, EVDS_VECTOR* v2);
// Find cross product between two vectors. Resulting vector is in coordinate system of v1
EVDS_API void EVDS_Vector_Cross(EVDS_VECTOR* target, EVDS_VECTOR* v1, EVDS_VECTOR* v2);
// Find dot product between two vectors in coordinate system of v1
EVDS_API void EVDS_Vector_Dot(EVDS_REAL* target, EVDS_VECTOR* v1, EVDS_VECTOR* v2);
// Normalize vector (returns direction)
EVDS_API void EVDS_Vector_Normalize(EVDS_VECTOR* target, EVDS_VECTOR* v);
// Return vector length
EVDS_API void EVDS_Vector_Length(EVDS_REAL* target, EVDS_VECTOR* v);
// Return distance between two vectors
EVDS_API void EVDS_Vector_Distance(EVDS_REAL* target, EVDS_VECTOR* v1, EVDS_VECTOR* v2);
// Check if two vectors are equal
EVDS_API int EVDS_Vector_Equal(EVDS_VECTOR* v1, EVDS_VECTOR* v2);

// Multiply vector by scalar
EVDS_API void EVDS_Vector_Multiply(EVDS_VECTOR* target, EVDS_VECTOR* v, EVDS_REAL scalar);
// Multiply and add
EVDS_API void EVDS_Vector_MultiplyAndAdd(EVDS_VECTOR* target, EVDS_VECTOR* source, EVDS_VECTOR* v, EVDS_REAL scalar);
// Multiply by time and add
EVDS_API void EVDS_Vector_MultiplyByTimeAndAdd(EVDS_VECTOR* target, EVDS_VECTOR* source, EVDS_VECTOR* v, EVDS_REAL delta_time);
// Interpolate vector linearly between two vectors.
EVDS_API void EVDS_Vector_Interpolate(EVDS_VECTOR* target, EVDS_VECTOR* v1, EVDS_VECTOR* v2, EVDS_REAL t);
// Multiply and add
EVDS_API void EVDS_StateVector_Derivative_MultiplyAndAdd(EVDS_STATE_VECTOR_DERIVATIVE* target, EVDS_STATE_VECTOR_DERIVATIVE* source, 
														 EVDS_STATE_VECTOR_DERIVATIVE* v, EVDS_REAL scalar);
// Multiply by time and add (all components of state vector at once)
EVDS_API void EVDS_StateVector_MultiplyByTimeAndAdd(EVDS_STATE_VECTOR* target, EVDS_STATE_VECTOR* source, 
													EVDS_STATE_VECTOR_DERIVATIVE* v, EVDS_REAL delta_time);
// Interpolate between two state vectors
EVDS_API void EVDS_StateVector_Interpolate(EVDS_STATE_VECTOR* target, EVDS_STATE_VECTOR* v1, EVDS_STATE_VECTOR* v2, EVDS_REAL t);

// Set euler angles in a target coordinate system
EVDS_API void EVDS_Quaternion_FromEuler(EVDS_QUATERNION* target, EVDS_OBJECT* target_coordinates, EVDS_REAL x, EVDS_REAL y, EVDS_REAL z);
// Get euler angles in a target coordinate system
EVDS_API void EVDS_Quaternion_ToEuler(EVDS_QUATERNION* q, EVDS_OBJECT* target_coordinates, EVDS_REAL* x, EVDS_REAL* y, EVDS_REAL* z);
// Set rotation vector and angle in target coordinate system
EVDS_API void EVDS_Quaternion_FromVectorAngle(EVDS_QUATERNION* target, EVDS_VECTOR* axis, EVDS_REAL angle);
// Get rotation vector and angle in target coordinate system
EVDS_API void EVDS_Quaternion_ToVectorAngle(EVDS_QUATERNION* q, EVDS_VECTOR* axis, EVDS_REAL* angle);
// Convert quaternion to a matrix
EVDS_API void EVDS_Quaternion_ToMatrix(EVDS_QUATERNION* q, EVDS_MATRIX m);
// Multiply two quaternions (must be specified in the same coordinate system)
EVDS_API void EVDS_Quaternion_Multiply(EVDS_QUATERNION* target, EVDS_QUATERNION* q, EVDS_QUATERNION* r);
// Multiply  conjugated first quaternion by second quaternion
EVDS_API void EVDS_Quaternion_MultiplyConjugatedQ(EVDS_QUATERNION* target, EVDS_QUATERNION* q, EVDS_QUATERNION* r);
// Multiply first quaternion by conjugated second quaternion
EVDS_API void EVDS_Quaternion_MultiplyConjugatedR(EVDS_QUATERNION* target, EVDS_QUATERNION* q, EVDS_QUATERNION* r);
// Multiply by a scalar
EVDS_API void EVDS_Quaternion_MultiplyScalar(EVDS_QUATERNION* target, EVDS_QUATERNION* source, EVDS_REAL scalar);
// Spherical quaternion interpolation.
EVDS_API void EVDS_Quaternion_Interpolate(EVDS_QUATERNION* target, EVDS_QUATERNION* q1, EVDS_QUATERNION* q2, EVDS_REAL t);
// Normalize quaternion to 1.0
EVDS_API void EVDS_Quaternion_Normalize(EVDS_QUATERNION* target, EVDS_QUATERNION* q);
// Rotate vector by quaternion. Vector will be converted to a coordinate system that is a child of quaternions coordinates
EVDS_API void EVDS_Vector_Rotate(EVDS_VECTOR* target, EVDS_VECTOR* v, EVDS_QUATERNION* q);
// Rotate vector by conjugate of quaternion (in opposite direction to what quaternion says)
EVDS_API void EVDS_Vector_RotateConjugated(EVDS_VECTOR* target, EVDS_VECTOR* v, EVDS_QUATERNION* q);

// Multiply 3x3 tensor (out of vectors) by a vector
EVDS_API void EVDS_Tensor_MultiplyByVector(EVDS_VECTOR* target, 
										   EVDS_VECTOR* mx, EVDS_VECTOR* my, EVDS_VECTOR* mz, EVDS_VECTOR* v);
// Rotate 3x3 tensor (out of vectors) by a quaternion
EVDS_API void EVDS_Tensor_Rotate(EVDS_VECTOR* tx, EVDS_VECTOR* ty, EVDS_VECTOR* tz,
								 EVDS_VECTOR* mx, EVDS_VECTOR* my, EVDS_VECTOR* mz,
								 EVDS_QUATERNION* q);
// Invert 3x3 tensor (symmetric moments of inertia tensor)
EVDS_API void EVDS_Tensor_InvertSymmetric(EVDS_VECTOR* tx, EVDS_VECTOR* ty, EVDS_VECTOR* tz,
										  EVDS_VECTOR* mx, EVDS_VECTOR* my, EVDS_VECTOR* mz);
// Invert 3x3 tensor (any tensor)
EVDS_API void EVDS_Tensor_Invert(EVDS_VECTOR* tx, EVDS_VECTOR* ty, EVDS_VECTOR* tz,
								 EVDS_VECTOR* mx, EVDS_VECTOR* my, EVDS_VECTOR* mz);
// Transpose a matrix
EVDS_API void EVDS_Matrix_Transpose(EVDS_MATRIX target, EVDS_MATRIX source);
// Check if tensor is symmetric
EVDS_API void EVDS_Tensor_IsSymmetric(EVDS_VECTOR* mx, EVDS_VECTOR* my, EVDS_VECTOR* mz, int* is_symmetric);

// Create new state vector
EVDS_API void EVDS_StateVector_Initialize(EVDS_STATE_VECTOR* v, EVDS_OBJECT* target_coordinates);
// Create new derivative of state vector
EVDS_API void EVDS_StateVector_Derivative_Initialize(EVDS_STATE_VECTOR_DERIVATIVE* v, EVDS_OBJECT* target_coordinates);

//
EVDS_API void EVDS_ShortVector_Add(EVDS_SHORT_VECTOR* target, EVDS_SHORT_VECTOR* v1, EVDS_SHORT_VECTOR* v2);
//
EVDS_API void EVDS_ShortVector_Subtract(EVDS_SHORT_VECTOR* target, EVDS_SHORT_VECTOR* v1, EVDS_SHORT_VECTOR* v2);
//
EVDS_API void EVDS_ShortVector_Cross(EVDS_SHORT_VECTOR* target, EVDS_SHORT_VECTOR* v1, EVDS_SHORT_VECTOR* v2);
//
EVDS_API void EVDS_ShortVector_Dot(EVDS_REAL* target, EVDS_SHORT_VECTOR* v1, EVDS_SHORT_VECTOR* v2);
//
EVDS_API void EVDS_ShortVector_Normalize(EVDS_SHORT_VECTOR* target, EVDS_SHORT_VECTOR* v);
//
EVDS_API void EVDS_ShortVector_Length(EVDS_REAL* target, EVDS_SHORT_VECTOR* v);
//
EVDS_API void EVDS_ShortVector_Distance(EVDS_REAL* target, EVDS_SHORT_VECTOR* v1, EVDS_SHORT_VECTOR* v2);
//
EVDS_API int EVDS_ShortVector_Equal(EVDS_SHORT_VECTOR* v1, EVDS_SHORT_VECTOR* v2);
//
EVDS_API void EVDS_ShortVector_Multiply(EVDS_SHORT_VECTOR* target, EVDS_SHORT_VECTOR* v, EVDS_REAL scalar);

////////////////////////////////////////////////////////////////////////////////
/// @}
////////////////////////////////////////////////////////////////////////////////








////////////////////////////////////////////////////////////////////////////////
/// @defgroup EVDS_ENVIRONMENT Environment API
/// @brief A set of default models and code to determine envrionment parameters
///
/// @{
////////////////////////////////////////////////////////////////////////////////
// Get acceleration due to gravity in the given position (local X Y Z acceleration, field)
EVDS_API int EVDS_Environment_GetGravitationalField(EVDS_SYSTEM* system, EVDS_VECTOR* position, EVDS_REAL* phi, EVDS_VECTOR* field);
// Get magnetic field vector in the given position (local X Y Z magnetic field)
EVDS_API int EVDS_Environment_GetMagneticField(EVDS_SYSTEM* system, EVDS_VECTOR* position, EVDS_VECTOR* field);
// Get atmospheric parameters (including contents by elements)
EVDS_API int EVDS_Environment_GetAtmosphericParameters(EVDS_SYSTEM* system, EVDS_VECTOR* position, EVDS_ENVIRONMENT_ATMOSPHERE* parameters);
// Get radiation intensity (including energy spectrum)
EVDS_API int EVDS_Environment_GetRadiationParameters(EVDS_SYSTEM* system, EVDS_VECTOR* position, EVDS_ENVIRONMENT_RADIATION* parameters);

// Play a sound in the 3D world
EVDS_API int EVDS_Sound_Play(EVDS_SYSTEM* system, EVDS_SOUND* sound);
// Get the next sound entry and remove it from the internal list of sounds
EVDS_API int EVDS_Sound_Pop(EVDS_SYSTEM* system, EVDS_SOUND* sound);
////////////////////////////////////////////////////////////////////////////////
/// @}
////////////////////////////////////////////////////////////////////////////////








////////////////////////////////////////////////////////////////////////////////
/// @defgroup EVDS_FRAME Frames of Reference API
/// @brief API for converting between special (non-cartesian) frames of reference
///
/// @{
////////////////////////////////////////////////////////////////////////////////
// Calculates datum based on object
EVDS_API void EVDS_Geodetic_DatumFromObject(EVDS_GEODETIC_DATUM* datum, EVDS_OBJECT* object);
// Set geodetic coordinate around object
EVDS_API void EVDS_Geodetic_Set(EVDS_GEODETIC_COORDINATE* coordinate, EVDS_OBJECT* object,
								EVDS_REAL latitude, EVDS_REAL longitude, EVDS_REAL elevation);
// Converts geodetic coordinates around object to a position vector
EVDS_API void EVDS_Geodetic_ToVector(EVDS_VECTOR* target, EVDS_GEODETIC_COORDINATE* source);
// Convert position vector to geodetic coordinates around object
EVDS_API void EVDS_Geodetic_FromVector(EVDS_GEODETIC_COORDINATE* target, EVDS_VECTOR* source, EVDS_GEODETIC_DATUM* target_datum);
// Return state vector of the LVLH frame
EVDS_API void EVDS_LVLH_GetStateVector(EVDS_STATE_VECTOR* target, EVDS_GEODETIC_COORDINATE* coordinate);
// Convert quaternion to objects LVLH frame
EVDS_API void EVDS_LVLH_QuaternionToLVLH(EVDS_QUATERNION* target_lvlh, EVDS_QUATERNION* source, EVDS_GEODETIC_COORDINATE* coordinate);
// Convert quaternion from objects LVLH frame
EVDS_API void EVDS_LVLH_QuaternionFromLVLH(EVDS_QUATERNION* target, EVDS_QUATERNION* source_lvlh, EVDS_GEODETIC_COORDINATE* coordinate);
////////////////////////////////////////////////////////////////////////////////
/// @}
////////////////////////////////////////////////////////////////////////////////








////////////////////////////////////////////////////////////////////////////////
/// @defgroup EVDS_MESH Mesh API
/// @brief A set of functions to tesselate procedural objects as 3D meshes
///
/// @{
////////////////////////////////////////////////////////////////////////////////
// Generate 3D mesh for the object
EVDS_API int EVDS_Mesh_Generate(EVDS_OBJECT* object, EVDS_MESH** p_mesh, float resolution, int flags);
// Generate 3D mesh for the object. Pass EVDS_GENERATEMESH_EX
EVDS_API int EVDS_Mesh_GenerateEx(EVDS_OBJECT* object, EVDS_MESH** p_mesh, EVDS_MESH_GENERATEEX* info);
// Destroy 3D mesh for the object
EVDS_API int EVDS_Mesh_Destroy(EVDS_MESH* mesh);
////////////////////////////////////////////////////////////////////////////////
/// @}
////////////////////////////////////////////////////////////////////////////////








////////////////////////////////////////////////////////////////////////////////
/// @page EVDS_Object_Types Object Types
///
/// The following object types are available in EVDS:
/// - @subpage EVDS_Solver_RigidBody "Rigid body/vessel"
///		Default model for a rigid body object, which accumulates forces and torques from children,
///		is affected by gravity and other effects (basic aerodynamic drag, etc).
/// - @subpage EVDS_Solver_RocketEngine "Rocket engine"
///		Rocket engine model which supports a variety of different parameters as input (physics
///		shape or properties).
/// - @subpage EVDS_Solver_Gimbal "Gimbal platform"
///		Allows controlled movement/rotation of children objects around origin of the platform.
/// - @subpage EVDS_Solver_FuelTank "Fuel tank"
///		Model of a fuel tank that stores liquid or solid fuel/oxidizer.
/// - @subpage EVDS_Solver_Wiring "Wiring/piping"
///		Wires or pipes, connecting nodes and connectors.
/// - @subpage EVDS_Solver_Planet "Planet/celestial body"
///		Planet or moon with built-in coordinate systems. Planetary position can be
///		updated from an ephemeris, orbital information, or be physically simulated.
/// - @subpage EVDS_Solver_Modifier "Modifier"
///		Creates copies of its children according to a predefined pattern.
///
///
/// The following propagators are available:
/// - @subpage EVDS_Propagator_ForwardEuler "Eulers forward integration" (for debugging purposes only)
/// - @subpage EVDS_Propagator_RK4 "Runge-Kutta 4th order integration"
/// - @subpage EVDS_Propagator_Heun "Heun's predictor-corrector integration"
////////////////////////////////////////////////////////////////////////////////
/// @page EVDS_Addon_List List of Addons
///
/// The following official addons are available for EVDS:
/// - @subpage EVDS_Solver_Antenna "Radio antenna"
///		Radio antenna with various shapes and geometric parameters available. If
///		Realtime Digital Radio Simulator support is enabled, it can be used for
///		simulating the digital radio link within the current EVDS_SYSTEM.
/// - @subpage EVDS_Solver_Train_WheelsGeometry "Train wheels (geometry only)"
///		Temporary placeholder that only represents geometry of train wheels.
/// - @subpage EVDS_Callback_NRLMSISE_00
///		NRLMSISE-00 Earth atmospheric model callback for the EVDS_ENVIRONMENT API.
/// - @subpage EVDS_Callback_WMM
///		World Magnetic Model callbacks for magnetic model of Earth.
////////////////////////////////////////////////////////////////////////////////








////////////////////////////////////////////////////////////////////////////////
/// @defgroup EVDS_ADDONS Addons API
/// @brief API for optional EVDS addons.
////////////////////////////////////////////////////////////////////////////////
/// @defgroup EVDS_OBJECTS Objects-specific API
/// @brief API for built-in objects (vessels, propagators, etc)
///
/// @{
////////////////////////////////////////////////////////////////////////////////
// Rigid body (moving object with mass), vessel (same as rigid body), vessel part (static object)
EVDS_API int EVDS_RigidBody_Register(EVDS_SYSTEM* system);
// Linear force or torque
EVDS_API int EVDS_Force_Register(EVDS_SYSTEM* system);

// Rocket engine (creates force along it's forward axis)
EVDS_API int EVDS_RocketEngine_Register(EVDS_SYSTEM* system);
// Fuel tank (stores fuel)
EVDS_API int EVDS_FuelTank_Register(EVDS_SYSTEM* system);
// Gimbal platform (allows turning position of set of objects in space)
EVDS_API int EVDS_Gimbal_Register(EVDS_SYSTEM* system);
// Planet (represents a planetary body or a star)
EVDS_API int EVDS_Planet_Register(EVDS_SYSTEM* system);
// Wiring, piping, connectors, etc
EVDS_API int EVDS_Wiring_Register(EVDS_SYSTEM* system);
// Modifier
EVDS_API int EVDS_Modifier_Register(EVDS_SYSTEM* system);
// Aerodynamic wing or aerodynamic surface
EVDS_API int EVDS_Wing_Register(EVDS_SYSTEM* system);

// Forward euler propagator
EVDS_API int EVDS_Propagator_ForwardEuler_Register(EVDS_SYSTEM* system);
// Heun propagator-corrector solver
EVDS_API int EVDS_Propagator_Heun_Register(EVDS_SYSTEM* system);
// Runge-Kutta 4th order propagator
EVDS_API int EVDS_Propagator_RK4_Register(EVDS_SYSTEM* system);

// Update all vessels and detach them if required. Must be called by user to support "detach" variable for vessels.
EVDS_API int EVDS_RigidBody_UpdateDetaching(EVDS_SYSTEM* system);
// Returns EVDS_OK if rigid body is consistent (solver was called at least once)
EVDS_API int EVDS_RigidBody_IsConsistent(EVDS_OBJECT* object);
// Get total mass of rigid body. If called upon a child, returns total mass of the parent including all children
EVDS_API int EVDS_RigidBody_GetTotalMass(EVDS_OBJECT* object, EVDS_REAL* p_mass);

// Consume fuel from a fuel tank (thread-safe - does not leak fuel away)
EVDS_API int EVDS_FuelTank_Consume(EVDS_OBJECT* tank, EVDS_REAL amount, EVDS_REAL* consumed);

// Find the nearest planetary body
EVDS_API int EVDS_Planet_GetNearest(EVDS_OBJECT* object, EVDS_OBJECT** p_planet);

// Check if material is oxidier
EVDS_API int EVDS_Material_IsOxidizer(EVDS_SYSTEM* system, const char* name);
// Check if material is fuel
EVDS_API int EVDS_Material_IsFuel(EVDS_SYSTEM* system, const char* name);


////////////////////////////////////////////////////////////////////////////////
/// Register all common objects
#define EVDS_Common_Register(system) \
EVDS_RigidBody_Register(system); \
EVDS_Force_Register(system); \
EVDS_RocketEngine_Register(system); \
EVDS_FuelTank_Register(system); \
EVDS_Gimbal_Register(system); \
EVDS_Planet_Register(system); \
EVDS_Wiring_Register(system); \
EVDS_Modifier_Register(system); \
EVDS_Propagator_ForwardEuler_Register(system); \
EVDS_Propagator_Heun_Register(system); \
EVDS_Propagator_RK4_Register(system);
////////////////////////////////////////////////////////////////////////////////
/// @}
////////////////////////////////////////////////////////////////////////////////

#ifdef __cplusplus
}
#endif
#endif
