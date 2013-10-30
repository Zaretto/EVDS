#include <stdio.h>
#include <math.h>
#include "evds.h"

int RK2_Solve(EVDS_SYSTEM* system, EVDS_SOLVER* solver, EVDS_OBJECT* coordinate_system, EVDS_REAL h) {
	SIMC_LIST* children;
	SIMC_LIST_ENTRY* entry;

	//Process all children
	EVDS_Object_GetChildren(coordinate_system,&children);
	entry = SIMC_List_GetFirst(children);
	while (entry) {
		//Initial state, final derivative:
		EVDS_STATE_VECTOR state;							//Initial state (t = 0)
		//Variables for RK2:
		EVDS_STATE_VECTOR state_temporary;					//Used in calculations
		EVDS_STATE_VECTOR_DERIVATIVE state_derivative_1;	//Two derivatives for RK2
		EVDS_STATE_VECTOR_DERIVATIVE state_derivative_2;
		EVDS_OBJECT* object = (EVDS_OBJECT*)SIMC_List_GetData(children,entry);

		// Solve everything inside the child
		if (EVDS_Object_Solve(object,h) != EVDS_OK) {
			// In case there is an error move to the next object in list.
			entry = SIMC_List_GetNext(children,entry);
			continue;
		}

		// Get initial state vector
		EVDS_Object_GetStateVector(object,&state);
		
		// f1 = f(0,y)
		EVDS_Object_Integrate(object,0.0,&state,&state_derivative_1);

		// f2 = f(t+0.5*h,y+0.5*h*f1)
		EVDS_StateVector_MultiplyByTimeAndAdd(&state_temporary,&state,&state_derivative_1,0.5*h);
		EVDS_Object_Integrate(object,0.5*h,&state_temporary,&state_derivative_2);

		// state = state + h*f2
		EVDS_StateVector_MultiplyByTimeAndAdd(&state,&state,&state_derivative_2,h);

		// Update object state vector
		EVDS_Object_SetStateVector(object,&state);

		//Move to next object in list
		entry = SIMC_List_GetNext(children,entry);
	}

	return EVDS_OK;
}

int RK2_Initialize(EVDS_SYSTEM* system, EVDS_SOLVER* solver, EVDS_OBJECT* object) {
	if (EVDS_Object_CheckType(object,"propagator_rk2") != EVDS_OK) return EVDS_IGNORE_OBJECT; 
	return EVDS_CLAIM_OBJECT;
}

EVDS_SOLVER Propagator_RK2 = {
	RK2_Initialize, //OnInitialize
	0, //OnDeinitialize
	RK2_Solve, //OnSolve
	0, //OnIntegrate
	0, //OnStateSave
	0, //OnStateLoad
	0, //OnStartup
	0, //OnShutdown
};

int RK2_Register(EVDS_SYSTEM* system) {
	return EVDS_Solver_Register(system,&Propagator_RK2);
}


void main() {
	EVDS_SYSTEM* system;
	EVDS_OBJECT* root;
	EVDS_OBJECT* inertial_system;
	EVDS_OBJECT* earth;
	EVDS_OBJECT* satellite;

	printf("Tutorial 2: Custom RK2 Propagator\n");
	EVDS_System_Create(&system);
	EVDS_Common_Register(system);
	RK2_Register(system);

	//Create inertial sysetm
	EVDS_System_GetRootInertialSpace(system,&root);
	EVDS_Object_Create(root,&inertial_system);
	EVDS_Object_SetType(inertial_system,"propagator_rk2");
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
