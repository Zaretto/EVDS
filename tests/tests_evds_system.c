#include "framework.h"

void Test_EVDS_SYSTEM() {
	START_TEST("Return codes") {
		EQUAL_TO(EVDS_System_Create(0), EVDS_ERROR_BAD_PARAMETER);
		EQUAL_TO(EVDS_System_Destroy(0), EVDS_ERROR_BAD_PARAMETER);
		EQUAL_TO(EVDS_System_SetTime(0,0.0), EVDS_ERROR_BAD_PARAMETER);
		EQUAL_TO(EVDS_System_GetTime(0,&real), EVDS_ERROR_BAD_PARAMETER);
		EQUAL_TO(EVDS_System_GetTime(system,0), EVDS_ERROR_BAD_PARAMETER);
		EQUAL_TO(EVDS_System_GetRootInertialSpace(0,&object), EVDS_ERROR_BAD_PARAMETER);
		EQUAL_TO(EVDS_System_GetRootInertialSpace(system,0), EVDS_ERROR_BAD_PARAMETER);
		EQUAL_TO(EVDS_System_GetObjectsByType(0,"test",&list), EVDS_ERROR_BAD_PARAMETER);
		EQUAL_TO(EVDS_System_GetObjectsByType(system,0,&list), EVDS_ERROR_BAD_PARAMETER);
		EQUAL_TO(EVDS_System_GetObjectsByType(system,"test",0), EVDS_ERROR_BAD_PARAMETER);
		EQUAL_TO(EVDS_System_GetObjectByUID(0,1234,0,&object), EVDS_ERROR_BAD_PARAMETER);
		EQUAL_TO(EVDS_System_GetObjectByUID(system,1234,0,0), EVDS_ERROR_BAD_PARAMETER);
		EQUAL_TO(EVDS_System_GetObjectByName(0,"test",0,&object), EVDS_ERROR_BAD_PARAMETER);
		EQUAL_TO(EVDS_System_GetObjectByName(system,0,0,&object), EVDS_ERROR_BAD_PARAMETER);
		EQUAL_TO(EVDS_System_GetObjectByName(system,"test",0,0), EVDS_ERROR_BAD_PARAMETER);
		EQUAL_TO(EVDS_System_CleanupObjects(0), EVDS_ERROR_BAD_PARAMETER);

		NEED_ARBITRARY_OBJECT();
		EQUAL_TO(EVDS_System_QueryByReference(object,0,0,0), EVDS_ERROR_BAD_PARAMETER);
		EQUAL_TO(EVDS_System_QueryByReference(0,"test",0,0), EVDS_ERROR_BAD_PARAMETER);

		EQUAL_TO(EVDS_System_DatabaseFromFile(system,0), EVDS_ERROR_BAD_PARAMETER);
		EQUAL_TO(EVDS_System_DatabaseFromFile(0,"test"), EVDS_ERROR_BAD_PARAMETER);
		EQUAL_TO(EVDS_System_DatabaseFromString(system,0), EVDS_ERROR_BAD_PARAMETER);
		EQUAL_TO(EVDS_System_DatabaseFromString(0,"test"), EVDS_ERROR_BAD_PARAMETER);
		EQUAL_TO(EVDS_System_GetDatabaseByName(0,"test",&variable), EVDS_ERROR_BAD_PARAMETER);
		EQUAL_TO(EVDS_System_GetDatabaseByName(system,0,&variable), EVDS_ERROR_BAD_PARAMETER);
		EQUAL_TO(EVDS_System_GetDatabaseByName(system,"test",0), EVDS_ERROR_BAD_PARAMETER);
		EQUAL_TO(EVDS_System_GetDatabasesList(0,&list), EVDS_ERROR_BAD_PARAMETER);
		EQUAL_TO(EVDS_System_GetDatabasesList(system,0), EVDS_ERROR_BAD_PARAMETER);
		EQUAL_TO(EVDS_System_GetDatabaseEntries(0,"test",&list), EVDS_ERROR_BAD_PARAMETER);
		EQUAL_TO(EVDS_System_GetDatabaseEntries(system,0,&list), EVDS_ERROR_BAD_PARAMETER);
		EQUAL_TO(EVDS_System_GetDatabaseEntries(system,"test",0), EVDS_ERROR_BAD_PARAMETER);
		//EQUAL_TO(EVDS_System_QueryDatabase(0,"test",&variable), EVDS_ERROR_BAD_PARAMETER);
		//EQUAL_TO(EVDS_System_QueryDatabase(system,0,&variable), EVDS_ERROR_BAD_PARAMETER);
		//EQUAL_TO(EVDS_System_QueryDatabase(system,"test",0), EVDS_ERROR_BAD_PARAMETER);
		EQUAL_TO(EVDS_System_SetGlobalCallbacks(0,0), EVDS_ERROR_BAD_PARAMETER);
		EQUAL_TO(EVDS_System_SetUserdata(0,0), EVDS_ERROR_BAD_PARAMETER);
		EQUAL_TO(EVDS_System_GetUserdata(system,0), EVDS_ERROR_BAD_PARAMETER);
		EQUAL_TO(EVDS_System_GetUserdata(0,&object), EVDS_ERROR_BAD_PARAMETER);
	} END_TEST


	START_TEST("Root inertial space") {
		ERROR_CHECK(EVDS_Object_Create(root,&object));

		EQUAL_TO(root->parent,0);
		EQUAL_TO(object->parent,root);
		IS_IN_LIST(object,root->raw_children);
		
		EQUAL_TO(root->state.position.coordinate_system,root);
		EQUAL_TO(root->state.velocity.coordinate_system,root);
		EQUAL_TO(root->state.angular_velocity.coordinate_system,root);
		EQUAL_TO(root->state.angular_acceleration.coordinate_system,root);
		EQUAL_TO(root->state.orientation.coordinate_system,root);

		ERROR_CHECK(EVDS_Object_GetParent(root,&object));
		EQUAL_TO(object,0);
	} END_TEST


	START_TEST("EVDS_System_CleanupObjects") {
		NEED_ARBITRARY_OBJECT();
		IS_IN_LIST(object,root->raw_children);

		ERROR_CHECK(EVDS_System_CleanupObjects(system)); //Will not delete object
		IS_IN_LIST(object,root->raw_children);
		IS_NOT_IN_LIST(object,system->deleted_objects);

		ERROR_CHECK(EVDS_Object_Destroy(object));
		EQUAL_TO(object->destroyed,1);
		IS_NOT_IN_LIST(object,root->raw_children);
		IS_IN_LIST(object,system->deleted_objects);

		ERROR_CHECK(EVDS_System_CleanupObjects(system)); //Will delete object
		IS_NOT_IN_LIST(object,system->deleted_objects);
	} END_TEST
}