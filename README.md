********************************************************************************
The EVDS library is still under development. Some features are still under development,
but it can already be used for familiarization.

Please refer to compilation guide on how to generate build files for target platform.
********************************************************************************




External Vessel Dynamics Simulator
--------------------------------------------------------------------------------
EVDS is an open-source aerospace physics simulation library (flight dynamics library, FDM)
written in C. It's designed for realtime and non-realtime, multithreaded simulation
of vessel dynamics. In the context of this library, "vessel" means any object to be
simulated, for example:
 - Autonomous spacecraft (satellites)
 - Manned spacecraft
 - Aircraft
 - Trains, vehicles

Conceptually, EVDS allows user to construct a set of equations for the entire
system from small basic equations (e.g. equations for numerical integration,
equations for aerodynamic forces). The equations are constructed from hierarchy
of objects, as defined by user.

Every object in EVDS has a coordinate system attached, and any vectors may be
specified in the objects coordinate system. Runtime conversion is performed
between vector coordinate systems.

A procedural approach to defining models is used: "it looks how it behaves".
The vessels are constructed from small pieces, each of which provides a contribution
to the total forces and torques or vessel behavior.

Every object contains a set of variables (floating-point, string, vector, quaternion
variables, or custom data with a pointer to arbitrary data structure). Custom
data variables may also contain nested attributes and variables (used for
storing complex structures in data files). A set of modifiers may be specified,
which create patterns/copies of objects according to given rules.

Vessel configuration can be defined in runtime by the program, or loaded from
an XML file.

EVDS does not contain a rendering engine, but provides facilities to generate
3D meshes and model data for rendering. The physics engine is designed for high-perfomance
multi-threaded environment with support for asynchronous rendering in mind.

EVDS contains a powerful vector math library which provides consistency checks
across different coordinate frames and allows correct transformation of vectors
between non-inertial coordinate frames.


Target Use
--------------------------------------------------------------------------------
EVDS was designed to be used in a realtime aerospace flight simulator, but
it can also be used for:
 - Quick first-order vessel parameter & motion estimation
 - Flight simulation
 - Physics calculations in embedded systems
 - Non-realtime simulation
 - Tessellation of 3D vessel models

The main version of EVDS is 64-bit, but it is also available in a 32-bit version.
EVDS supports the following platforms at the moment:
 - Windows (32-bit and 64-bit)
 - Linux (32-bit and 64-bit)
 
 
References
--------------------------------------------------------------------------------
Links:
 - [EVDS Documentation (http://evds.wireos.com)](http://evds.wireos.com/).
 - [Black Phoenix (phoenix@uol.ua)](mailto:phoenix@uol.ua).
 - [Black Phoenix's Website (http://brain.wireos.com)](http://brain.wireos.com/).

Community:
 - IRC: #x-plane @ irc.x-plane.org
 - IRC: #foxworks @ irc.freenode.org
 - IRC: ##aerospace @ irc.freenode.org


Compiling
--------------------------------------------------------------------------------
All the EVDS dependencies (SIMC, TinyXML) should be downloaded automatically
with the Git repository, otherwise use the following command to include submodules:
```
git clone --recursive https://github.com/FoxWorks/EVDS.git
```

The EVDS library contains build files for various platforms in the `support`
folder:
 - `vs2008` - Visual Studio 2008
 - `vs2010` - Visual Studio 2010
 - `gmake` - Linux Makefiles

If build files must be generated from scratch or a local copy of documentation
is needed, [Premake5 or Premake4.4](http://industriousone.com/premake) must be used:
```
cd support
premake4 vs2008
premake4 vs2010
premake4 gmake
premake4 evdsdoc
```

Use the generated `sln` files under Windows. They will include all required
dependencies (which are present as submodules in repository).

Use makefiles under Linux or other platforms:
```
cd support/gmake
make
```


Example of Use
--------------------------------------------------------------------------------
[Documentation for the EVDS library is available here](http://evds.wireos.com/).

The EVDS library is hopefully easy to use, but requires at least basic knowledge
of aerospace topics. Here's a sample code for a small simulator:
```c
void main() {
	EVDS_SYSTEM* system;
	EVDS_OBJECT* inertial_system;
	EVDS_OBJECT* earth;
	EVDS_OBJECT* satellite;

	EVDS_System_Create(&system);
	EVDS_Common_Register(system);

	//Create inertial system/propagator
	EVDS_Object_Create(system,0,&inertial_system);
	EVDS_Object_SetType(inertial_system,"propagator_rk4");
	EVDS_Object_Initialize(inertial_system,1);

	//Create planet Earth
	EVDS_Object_Create(system,inertial_system,&earth);
	EVDS_Object_SetType(earth,"planet");
	EVDS_Object_SetName(earth,"Earth");
	EVDS_Object_AddFloatVariable(earth,"mu",3.9860044e14,0);    //m3 sec-2
	EVDS_Object_AddFloatVariable(earth,"radius",6378.145e3,0);  //m
	EVDS_Object_AddFloatVariable(earth,"period",86164.10,0);    //sec
	EVDS_Object_SetPosition(earth,inertial_system,0,0,0);
	EVDS_Object_Initialize(earth,1);

	//Load satellite
	EVDS_Object_LoadFromFile(inertial_system,"satellite.evds",&satellite);
	EVDS_Object_SetPosition(satellite,inertial_system,6728e3,0,0);
	EVDS_Object_SetVelocity(satellite,inertial_system,0,7700,0);
	EVDS_Object_Initialize(satellite,1);

	//Main simulation loop
	while (1) {
		//Propagate state
		EVDS_Object_Solve(inertial_system,1.0);

		//Add a small delay
		SIMC_Thread_Sleep(0.02);
	}

	EVDS_System_Destroy(system);
}
```

Development Plans
--------------------------------------------------------------------------------
These are the features that are yet to be implemented in the core EVDS library or
addons:
- Find some interesting people to discuss various function names and variable
 names with to finalize the API before first release.
- A way to specify explicitly which variables are interpolated by.
- If variable already exists with wrong type, many functions will silently fail
 without any way to find out exact reason
- Add additional debug asserts in various code points to avoid common errors.
- Unique identifiers must be made truly unique
- Add more unit tests
- Work more on predefined coordinate frames
- Add proper support for geodetic datums
- Basic aerodynamics code
