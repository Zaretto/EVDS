#include "framework.h"

void Test_EVDS_VECTOR() {
	START_TEST("Torque and force") {
		EVDS_VECTOR new_position,force,torque;

		//
		EVDS_Vector_Set(&force, EVDS_VECTOR_FORCE, system->inertial_space, 100.0, 0.0, 0.0);
		EVDS_Vector_SetPosition(&force, system->inertial_space, 0.0, 0.0, -100.0);
		EVDS_Vector_Set(&new_position, EVDS_VECTOR_POSITION, system->inertial_space, 0.0, 0.0, 0.0);
		EVDS_Vector_MoveForceToPosition(&force, &torque, &new_position);

		//
		EVDS_Vector_Set(&force, EVDS_VECTOR_FORCE, system->inertial_space, 100.0, 0.0, 0.0);
		EVDS_Vector_SetPosition(&force, system->inertial_space, 0.0, 0.0, 100.0);
		EVDS_Vector_Set(&new_position, EVDS_VECTOR_POSITION, system->inertial_space, 0.0, 0.0, 0.0);
		EVDS_Vector_MoveForceToPosition(&force, &torque, &new_position);

		//
		EVDS_Vector_Set(&force, EVDS_VECTOR_FORCE, system->inertial_space, 100.0, 0.0, 0.0);
		EVDS_Vector_SetPosition(&force, system->inertial_space, 0.0, 0.0, 100.0);
		EVDS_Vector_Set(&new_position, EVDS_VECTOR_POSITION, system->inertial_space, 0.0, 0.0, 0.0);
		EVDS_Vector_MoveForceToPosition(&force, &torque, &new_position);
	} END_TEST


	START_TEST("Handedness tests") {
		//Behavior: pitch forward vector up by 90 deg
		EVDS_Vector_Set(&vector,EVDS_VECTOR_POSITION,system->inertial_space,-1.0,0.0,0.0);
		EVDS_Quaternion_FromEuler(&quaternion,system->inertial_space,EVDS_RAD(0.0),EVDS_RAD(90.0),EVDS_RAD(0.0));
		EVDS_Vector_Rotate(&vector,&vector,&quaternion);
		VECTOR_EQUAL_TO(&vector,0.0,0.0,1.0);

		//Behavior: yaw aft vector (left) by 90 deg
		EVDS_Vector_Set(&vector,EVDS_VECTOR_POSITION,system->inertial_space,1.0,0.0,0.0);
		EVDS_Quaternion_FromEuler(&quaternion,system->inertial_space,EVDS_RAD(0.0),EVDS_RAD(0.0),EVDS_RAD(90.0));
		EVDS_Vector_Rotate(&vector,&vector,&quaternion);
		VECTOR_EQUAL_TO(&vector,0.0,1.0,0.0);

		//Behavior: roll right vector by 90 deg (roll left)
		EVDS_Vector_Set(&vector,EVDS_VECTOR_POSITION,system->inertial_space,0.0,1.0,0.0);
		EVDS_Quaternion_FromEuler(&quaternion,system->inertial_space,EVDS_RAD(90.0),EVDS_RAD(0.0),EVDS_RAD(0.0));
		EVDS_Vector_Rotate(&vector,&vector,&quaternion);
		VECTOR_EQUAL_TO(&vector,0.0,0.0,1.0);

		//Test cross product (y x z = x)
		EVDS_Vector_Set(&vector,EVDS_VECTOR_POSITION,system->inertial_space,0.0,1.0,0.0);
		EVDS_Vector_Set(&vector2,EVDS_VECTOR_POSITION,system->inertial_space,0.0,0.0,1.0);
		EVDS_Vector_Cross(&vector,&vector,&vector2);
		VECTOR_EQUAL_TO(&vector,1.0,0.0,0.0);
	} END_TEST


	/*START_TEST("Nested transformations") {
		EVDS_Vector_Set(&vessel->state.position,			EVDS_VECTOR_POSITION,			inertial, 100.0, 0.0, 0.0);
		EVDS_Vector_Set(&vessel->state.velocity,			EVDS_VECTOR_VELOCITY,			inertial, 0.0,   0.0, 0.0);
		EVDS_Vector_Set(&vessel->state.angular_velocity,	EVDS_VECTOR_ANGULAR_VELOCITY,	inertial, 0.0,   0.0, 0.0);
		EVDS_Quaternion_SetEuler(&vessel->state.orientation,								inertial, 0.0, 0.0, 0.0);

		EVDS_Vector_Set(&nested_vessel->state.position,			EVDS_VECTOR_POSITION,			vessel, 100.0, 0.0, 0.0);
		EVDS_Vector_Set(&nested_vessel->state.velocity,			EVDS_VECTOR_VELOCITY,			vessel, 0.0,   0.0, 0.0);
		EVDS_Vector_Set(&nested_vessel->state.angular_velocity,	EVDS_VECTOR_ANGULAR_VELOCITY,	vessel, 0.0,   0.0, 0.0);
		EVDS_Quaternion_SetEuler(&nested_vessel->state.orientation,								vessel, 0.0, 0.0, 0.0);

		EVDS_Vector_Set(&nested_vessel2->state.position,			EVDS_VECTOR_POSITION,			inertial, 50.0, 0.0, 0.0);
		EVDS_Vector_Set(&nested_vessel2->state.velocity,			EVDS_VECTOR_VELOCITY,			inertial, 0.0,   0.0, 0.0);
		EVDS_Vector_Set(&nested_vessel2->state.angular_velocity,	EVDS_VECTOR_ANGULAR_VELOCITY,	inertial, 0.0,   0.0, 0.0);
		EVDS_Quaternion_SetEuler(&nested_vessel2->state.orientation,								inertial, 0.0, 0.0, 0.0);

		//----------------------------------------------------------------------
		//Behavior: inertial point is 200 meters in front of the vessel
		EVDS_Vector_Set(&local_position,EVDS_VECTOR_POSITION,nested_vessel,-200.0,0.0,0.0);
		EVDS_Vector_Convert(&inertial_position,&local_position,inertial);
		VECTOR_TEST(&inertial_position,0.0,0.0,0.0);

		//Behavior: inertial point is 200 meters in front of the vessel
		EVDS_Vector_Set(&inertial_position,EVDS_VECTOR_POSITION,inertial,0.0,0.0,0.0);
		EVDS_Vector_Convert(&local_position,&inertial_position,nested_vessel);
		VECTOR_TEST(&local_position,-200.0,0.0,0.0);

		//Behavior: inertial point is 200 meters in front of the vessel. It's 50 meters in front of the nested_vessel2
		EVDS_Vector_Set(&local_position,EVDS_VECTOR_POSITION,nested_vessel,-200.0,0.0,0.0);
		EVDS_Vector_Convert(&vector,&local_position,nested_vessel2);
		VECTOR_TEST(&vector,-50.0,0.0,0.0);
		
		//FIXME: add more checks
	} END_TEST


	//--------------------------------------------------------------------------
	//Check converting between local and inertial coordinates
	if (1) {
		EVDS_Vector_Set(&vessel->state.position,			EVDS_VECTOR_POSITION,			inertial, 100.0, 0.0, 0.0);
		EVDS_Vector_Set(&vessel->state.velocity,			EVDS_VECTOR_VELOCITY,			inertial, 0.0,   0.0, 0.0);
		EVDS_Vector_Set(&vessel->state.angular_velocity,	EVDS_VECTOR_ANGULAR_VELOCITY,	inertial, 0.0,   0.0, 0.0);
		EVDS_Quaternion_SetEuler(&vessel->state.orientation,								inertial, 0.0, 0.0, 0.0);

		//----------------------------------------------------------------------
		//No vessel rotation (from inertial to local)
		//Behavior: inertial point is 1 meter behind of vessel
		EVDS_Vector_Set(&inertial_position,EVDS_VECTOR_POSITION,inertial,101.0,0.0,0.0);
		EVDS_Vector_Convert(&local_position,&inertial_position,vessel);
		VECTOR_TEST(&local_position,1.0,0.0,0.0);

		//Behavior: inertial point is 1 meter left of vessel
		EVDS_Vector_Set(&inertial_position,EVDS_VECTOR_POSITION,inertial,100.0,1.0,0.0);
		EVDS_Vector_Convert(&local_position,&inertial_position,vessel);
		VECTOR_TEST(&local_position,0.0,1.0,0.0);

		//Behavior: inertial point is 1 meter above vessel
		EVDS_Vector_Set(&inertial_position,EVDS_VECTOR_POSITION,inertial,100.0,0.0,1.0);
		EVDS_Vector_Convert(&local_position,&inertial_position,vessel);
		VECTOR_TEST(&local_position,0.0,0.0,1.0);

		//----------------------------------------------------------------------
		//No vessel rotation (from local to inertial)
		//Behavior: local point is 1 meter behind of vessel
		EVDS_Vector_Set(&local_position,EVDS_VECTOR_POSITION,vessel,1.0,0.0,0.0);
		EVDS_Vector_Convert(&inertial_position,&local_position,inertial);
		VECTOR_TEST(&inertial_position,101.0,0.0,0.0);

		//Behavior: local point is 1 meter right
		EVDS_Vector_Set(&local_position,EVDS_VECTOR_POSITION,vessel,0.0,1.0,0.0);
		EVDS_Vector_Convert(&inertial_position,&local_position,inertial);
		VECTOR_TEST(&inertial_position,100.0,1.0,0.0);

		//Behavior: local point is 1 meter below
		EVDS_Vector_Set(&local_position,EVDS_VECTOR_POSITION,vessel,0.0,0.0,-1.0);
		EVDS_Vector_Convert(&inertial_position,&local_position,inertial);
		VECTOR_TEST(&inertial_position,100.0,0.0,-1.0);

		//----------------------------------------------------------------------
		//Vessel rotation present (from inertial to local, local to inertial)
		//Behavior: local point is 1 meter above of vessel
		//Result: pitching up by 90 deg will place it where vessels aft originally was
		EVDS_Quaternion_SetEuler(&vessel->state.orientation, inertial, 0.0, RAD(90.0), 0.0);
		EVDS_Vector_Set(&local_position,EVDS_VECTOR_POSITION,vessel,0.0,0.0,1.0);
		EVDS_Vector_Convert(&inertial_position,&local_position,inertial);
		VECTOR_TEST(&inertial_position,101.0,0.0,0.0);

		//Behavior: inertial point is 1 meter forward of vessel. Vessel pitched up by 90 degrees
		//Result: in local coordinates point is 1 meter below vessel
		EVDS_Quaternion_SetEuler(&vessel->state.orientation, inertial, 0.0, RAD(90.0), 0.0);
		EVDS_Vector_Set(&inertial_position,EVDS_VECTOR_POSITION,inertial,101.0,0.0,0.0);
		EVDS_Vector_Convert(&local_position,&inertial_position,vessel);
		VECTOR_TEST(&local_position,0.0,0.0,1.0);
	}
	

	//--------------------------------------------------------------------------
	//Check converting velocity
	if (1) {
		EVDS_Vector_Set(&vessel->state.position,			EVDS_VECTOR_POSITION,			inertial, 100.0, 0.0, 0.0);
		EVDS_Vector_Set(&vessel->state.velocity,			EVDS_VECTOR_VELOCITY,			inertial, 1.0,   0.0, 0.0);
		EVDS_Vector_Set(&vessel->state.angular_velocity,	EVDS_VECTOR_ANGULAR_VELOCITY,	inertial, 0.0,   0.0, 0.0);
		EVDS_Quaternion_SetEuler(&vessel->state.orientation,								inertial, 0.0,   0.0, 0.0);

		//----------------------------------------------------------------------
		//Behavior: zero velocity in inertial coordinates while local coordinates are moving
		EVDS_Vector_Set(&vector,EVDS_VECTOR_VELOCITY,inertial,0.0,0.0,0.0);
		EVDS_Vector_Convert(&vector,&vector,vessel);
		VECTOR_TEST(&vector,-1.0,0.0,0.0);

		//Behavior: zero velocity in local coordinates while local coordinates are moving
		EVDS_Vector_Set(&vector,EVDS_VECTOR_VELOCITY,vessel,0.0,0.0,0.0);
		EVDS_Vector_Convert(&vector,&vector,inertial);
		VECTOR_TEST(&vector,1.0,0.0,0.0);

		//----------------------------------------------------------------------
		//Behavior: coriolis rotation
		EVDS_Vector_Set(&vessel->state.position,			EVDS_VECTOR_POSITION,			inertial, 100.0, 0.0, 0.0);
		EVDS_Vector_Set(&vessel->state.velocity,			EVDS_VECTOR_VELOCITY,			inertial, 0.0,   0.0, 0.0);
		EVDS_Vector_Set(&vessel->state.angular_velocity,	EVDS_VECTOR_ANGULAR_VELOCITY,	inertial, 0.0,   1.0, 0.0);

		//Behavior: zero velocity in inertial coordinates at (0,0,0)
		EVDS_Vector_Set(&vector,EVDS_VECTOR_VELOCITY,inertial,0.0,0.0,0.0);
		EVDS_Vector_SetPositionVector(&vector,&origin);
		EVDS_Vector_Convert(&vector,&vector,vessel);
		VECTOR_TEST(&vector,0.0,0.0,-100.0);

		//Behavior: zero velocity in local coordinates, at vessel origin
		EVDS_Vector_Set(&vector,EVDS_VECTOR_VELOCITY,vessel,0.0,0.0,0.0);
		EVDS_Vector_SetPositionVector(&vector,&vessel->state.position);
		EVDS_Vector_Convert(&vector,&vector,inertial);
		VECTOR_TEST(&vector,0.0,0.0,0.0);

		//Behavior: zero velocity in local coordinates, offset (by 10 meters aft)
		EVDS_Vector_Set(&vector,EVDS_VECTOR_VELOCITY,vessel,0.0,0.0,0.0);
		EVDS_Vector_SetPositionVector(&vector,&vessel_test_position);
		EVDS_Vector_Convert(&vector,&vector,inertial);
		VECTOR_TEST(&vector,0.0,0.0,-10.0);
	}


	//--------------------------------------------------------------------------
	//Check converting in really messed up frames
	if (1) {
		EVDS_Vector_Set(&vessel->state.position,			EVDS_VECTOR_POSITION,					inertial, 0.0, 0.0, 0.0);
		EVDS_Vector_Set(&vessel->state.velocity,			EVDS_VECTOR_VELOCITY,					inertial, 0.0, 0.0, 0.0);
		EVDS_Vector_Set(&vessel->state.angular_velocity,	EVDS_VECTOR_ANGULAR_VELOCITY,			inertial, 0.0, 1.0, 0.0);
		EVDS_Quaternion_SetEuler(&vessel->state.orientation,										inertial, 0.0, 0.0, 0.0);

		//----------------------------------------------------------------------
		//Behavior: zero acceleration at 1 m behind vessel: centripetal acceleration towards center of rotation (0,0,0)
		EVDS_Vector_Set(&vector,EVDS_VECTOR_ACCELERATION,inertial,0.0,0.0,0.0);
		EVDS_Vector_SetPosition(&vector,inertial,1.0,0.0,0.0);
		EVDS_Vector_Convert(&vector,&vector,vessel);
		VECTOR_TEST(&vector,1.0,0.0,0.0);

		//Behavior: ...
		EVDS_Vector_Set(&vector,EVDS_VECTOR_ACCELERATION,inertial,0.0,0.0,0.0);
		EVDS_Vector_SetPosition(&vector,inertial,1.0,0.0,0.0);
		EVDS_Vector_SetVelocity(&vector,inertial,1.0,0.0,0.0);
		EVDS_Vector_Convert(&vector,&vector,vessel);
		VECTOR_TEST(&vector,1.0,0.0,2.0);
	}*/
}