#include <stdio.h>
#include <math.h>
#include "evds.h"

void main() {
	EVDS_SYSTEM* system;
	EVDS_OBJECT* inertial_system;
	EVDS_OBJECT* earth;
	EVDS_OBJECT* satellite;

	printf("Tutorial 1: 6-DOF Earth Satellite Model\n");
	EVDS_System_Create(&system);
	EVDS_Common_Register(system);

	//Create inertial system
	EVDS_System_GetRootInertialSpace(system, &inertial_system);
	EVDS_Object_SetType(inertial_system,"propagator_rk4");
	EVDS_Object_Initialize(inertial_system,1);

	//Create planet Earth
	EVDS_Object_Create(inertial_system,&earth);
	EVDS_Object_SetType(earth,"planet");
	EVDS_Object_SetName(earth,"Earth");
	EVDS_Object_AddRealVariable(earth,"mu",3.9860044e14,0);    //m3 sec-2
	EVDS_Object_AddRealVariable(earth,"radius",6378.145e3,0);  //m
	EVDS_Object_AddRealVariable(earth,"period",86164.10,0);    //sec
	EVDS_Object_SetPosition(earth,inertial_system,0,0,0);
	EVDS_Object_Initialize(earth,1);

	//Load satellite
	EVDS_Object_LoadFromString(inertial_system,
"<EVDS>"
"  <object type=\"vessel\" name=\"Satellite\">"
"    <parameter name=\"mass\">1000</parameter>"
"    <parameter name=\"ixx\">100</parameter>"
"    <parameter name=\"iyy\">1000</parameter>"
"    <parameter name=\"izz\">500</parameter>"
"    <parameter name=\"cm\">1.0 0.0 -0.5</parameter>"
"  </object>"
"</EVDS>",&satellite);
	EVDS_Object_SetPosition(satellite,inertial_system,6728e3,0,0);
	EVDS_Object_SetVelocity(satellite,inertial_system,0,7700,0);
	EVDS_Object_Initialize(satellite,1);

	//Main simulation loop
	while (1) {
		EVDS_STATE_VECTOR state;
		EVDS_REAL radius2,velocity2;

		//Propagate state
		EVDS_Object_Solve(inertial_system,1.0);
		
		//Print state and add a small delay
		EVDS_Object_GetStateVector(satellite,&state);
		EVDS_Vector_Dot(&radius2,&state.position,&state.position);
		EVDS_Vector_Dot(&velocity2,&state.velocity,&state.velocity);
		printf("MJD: %.5f  [%.0f %.0f] H = %.3f km  [%.0f %.0f] V = %.3f m/s]\r",
			state.time,
			state.position.x*1e-3,state.position.y*1e-3,sqrt(radius2)*1e-3-6378.0,
			state.velocity.x,state.velocity.y,sqrt(velocity2));
		SIMC_Thread_Sleep(0.02);
	}

	EVDS_System_Destroy(system);
}
