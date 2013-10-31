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
		EQUAL_TO(EVDS_System_GetObjectByUID(0,0,1234,&object), EVDS_ERROR_BAD_PARAMETER);
		EQUAL_TO(EVDS_System_GetObjectByUID(system,0,1234,0), EVDS_ERROR_BAD_PARAMETER);
		EQUAL_TO(EVDS_System_GetObjectByName(0,0,"test",&object), EVDS_ERROR_BAD_PARAMETER);
		EQUAL_TO(EVDS_System_GetObjectByName(system,0,0,&object), EVDS_ERROR_BAD_PARAMETER);
		EQUAL_TO(EVDS_System_GetObjectByName(system,0,"test",0), EVDS_ERROR_BAD_PARAMETER);
		EQUAL_TO(EVDS_System_CleanupObjects(0), EVDS_ERROR_BAD_PARAMETER);

		NEED_ARBITRARY_OBJECT();
		//EQUAL_TO(EVDS_System_QueryByReference(0,object,0,0,0), EVDS_ERROR_BAD_PARAMETER);
		//EQUAL_TO(EVDS_System_QueryByReference(0,0,"test",0,0), EVDS_ERROR_BAD_PARAMETER);

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
		

	START_TEST("EVDS_System_QueryByReference") {
		EVDS_OBJECT *obj;
		EVDS_VARIABLE *var;
		EVDS_OBJECT *nested_object;
		EVDS_VARIABLE *nested_variable;
		EVDS_VARIABLE *e1,*e2,*e3;
		EVDS_VARIABLE *d1,*d2,*d3;

		NEED_ARBITRARY_OBJECT();
		ERROR_CHECK(EVDS_Object_Create(object,&nested_object));

		ERROR_CHECK(EVDS_Object_SetName(object,"object_name"));
		ERROR_CHECK(EVDS_Object_SetName(nested_object,"nested_object"));
		ERROR_CHECK(EVDS_Object_AddVariable(object,"variable_name",EVDS_VARIABLE_TYPE_NESTED,&variable));
		ERROR_CHECK(EVDS_Variable_AddNested(variable,"nested_variable_name",EVDS_VARIABLE_TYPE_NESTED,&nested_variable));
		ERROR_CHECK(EVDS_Variable_AddNested(variable,"data1",EVDS_VARIABLE_TYPE_NESTED,&d1));
		ERROR_CHECK(EVDS_Variable_AddNested(variable,"data2",EVDS_VARIABLE_TYPE_NESTED,&d2));
		ERROR_CHECK(EVDS_Variable_AddNested(variable,"data3",EVDS_VARIABLE_TYPE_NESTED,&d3));
		ERROR_CHECK(EVDS_Variable_AddNested(nested_variable,"extra",EVDS_VARIABLE_TYPE_NESTED,&e1));
		ERROR_CHECK(EVDS_Variable_AddNested(nested_variable,"extra",EVDS_VARIABLE_TYPE_NESTED,&e2));
		ERROR_CHECK(EVDS_Variable_AddNested(nested_variable,"extra",EVDS_VARIABLE_TYPE_NESTED,&e3));

		//Run test queries
		ERROR_CHECK(EVDS_System_QueryByReference(system,0,"/object_name",&var,&obj));
		EQUAL_TO(var,0);
		EQUAL_TO(obj,object);

		ERROR_CHECK(EVDS_System_QueryByReference(system,0,"/object_name/nested_object",&var,&obj));
		EQUAL_TO(var,0);
		EQUAL_TO(obj,nested_object);

		EQUAL_TO(EVDS_System_QueryByReference(system,0,"/object_name/nested_object/non-existant_object",&var,&obj),EVDS_ERROR_NOT_FOUND);

		ERROR_CHECK(EVDS_System_QueryByReference(system,0,"/object_name/variable_name",&var,&obj));
		EQUAL_TO(var,variable);
		EQUAL_TO(obj,0);

		ERROR_CHECK(EVDS_System_QueryByReference(system,0,"/object_name/variable_name/nested_variable_name",&var,&obj));
		EQUAL_TO(var,nested_variable);
		EQUAL_TO(obj,0);


		//Query arrays of variables
		/// @c /vessel/geometry.cross_sections/section[5]
		/// @c /vessel/geometry.cross_sections[5]
		///	@c /vessel/geometry.cross_sections[5]/offset


		//Query by wildcards
		/// @c /*/object_name
		/// @c /*/variable_name


		//Query database
		EQUAL_TO(EVDS_System_QueryByReference(system,0,"[]",&var,&obj),EVDS_ERROR_NOT_FOUND);

		ERROR_CHECK(EVDS_System_QueryByReference(system,0,"[material]/N2O4",&var,&obj));
		STRING_EQUAL_TO(var->name,"N2O4");
		EQUAL_TO(obj,0);

		ERROR_CHECK(EVDS_System_QueryByReference(system,0,"[material]/H2/density",&var,&obj));
		STRING_EQUAL_TO(var->name,"density");
		EQUAL_TO(var->type,EVDS_VARIABLE_TYPE_FUNCTION);
		EQUAL_TO(obj,0);
	} END_TEST
}