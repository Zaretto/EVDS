#include "framework.h"

void Test_EVDS_MODIFIER() {
	START_TEST("Linear modifier test") {
		/// This test verifies that linear modifier creates children copies,
		/// and that these copies are named correctly and present at their correct spots.
		ERROR_CHECK(EVDS_Object_LoadFromString(root,
"<EVDS version=\"31\">"
"    <object name=\"Modifier\" type=\"modifier\" x=\"10\" y=\"20\" z=\"30\">"
"        <parameter name=\"pattern\">linear</parameter>"
"        <parameter name=\"vector1.count\">6</parameter>"
"        <parameter name=\"vector2.count\">5</parameter>"
"        <parameter name=\"vector3.count\">5</parameter>"
"        <parameter name=\"vector1.x\">1</parameter>"
"        <parameter name=\"vector2.y\">2</parameter>"
"        <parameter name=\"vector3.z\">3</parameter>"
"        <object name=\"Object\" type=\"static_body\">"
"            <parameter name=\"mass\">100</parameter>"
"        </object>"
"    </object>"
"    <object name=\"Object (3x3x3)\" type=\"static_body\" />"
"</EVDS>",&object));
		ERROR_CHECK(EVDS_Object_Initialize(object,1));

		/// Check if original object becomes part of the modifiers children container
		EQUAL_TO(EVDS_System_GetObjectByName(system,"Object",0,&object), EVDS_OK);
		EQUAL_TO(object->parent,root);
		EQUAL_TO(object->initialized,1);

		/// Check some key objects that must be part of the container
		EQUAL_TO(EVDS_System_GetObjectByName(system,"Object",root,&object), EVDS_OK);
		EQUAL_TO(object->initialized,1);
		VECTOR_EQUAL_TO(&object->state.position, 10+1*0, 20+2*0, 30+3*0);

		EQUAL_TO(EVDS_System_GetObjectByName(system,"Object (5x1x1)",root,&object), EVDS_OK);
		EQUAL_TO(object->initialized,1);
		VECTOR_EQUAL_TO(&object->state.position, 10+1*4, 20+2*0, 30+3*0);

		EQUAL_TO(EVDS_System_GetObjectByName(system,"Object (1x5x1)",root,&object), EVDS_OK);
		EQUAL_TO(object->initialized,1);
		VECTOR_EQUAL_TO(&object->state.position, 10+1*0, 20+2*4, 30+3*0);

		EQUAL_TO(EVDS_System_GetObjectByName(system,"Object (5x5x1)",root,&object), EVDS_OK);
		EQUAL_TO(object->initialized,1);
		VECTOR_EQUAL_TO(&object->state.position, 10+1*4, 20+2*4, 30+3*0);

		EQUAL_TO(EVDS_System_GetObjectByName(system,"Object (1x1x5)",root,&object), EVDS_OK);
		EQUAL_TO(object->initialized,1);
		VECTOR_EQUAL_TO(&object->state.position, 10+1*0, 20+2*0, 30+3*4);

		EQUAL_TO(EVDS_System_GetObjectByName(system,"Object (5x1x5)",root,&object), EVDS_OK);
		EQUAL_TO(object->initialized,1);
		VECTOR_EQUAL_TO(&object->state.position, 10+1*4, 20+2*0, 30+3*4);

		EQUAL_TO(EVDS_System_GetObjectByName(system,"Object (1x5x5)",root,&object), EVDS_OK);
		EQUAL_TO(object->initialized,1);
		VECTOR_EQUAL_TO(&object->state.position, 10+1*0, 20+2*4, 30+3*4);

		EQUAL_TO(EVDS_System_GetObjectByName(system,"Object (5x5x5)",root,&object), EVDS_OK);
		EQUAL_TO(object->initialized,1);
		VECTOR_EQUAL_TO(&object->state.position, 10+1*4, 20+2*4, 30+3*4);

		/// Check if special pre-defined object is not overwritten by the modifier
		EQUAL_TO(EVDS_System_GetObjectByName(system,"Object (3x3x3)",root,&object), EVDS_OK);
		EQUAL_TO(object->initialized,1);
		VECTOR_EQUAL_TO(&object->state.position, 0, 0, 0);
	} END_TEST


	START_TEST("Circular modifier test") {
		int i,j,k;

		/// This test does same as for linear modifier, but now using circular modifier.
		ERROR_CHECK(EVDS_Object_LoadFromString(root,
"<EVDS version=\"31\">"
"    <object name=\"Modifier\" type=\"modifier\">"
"        <parameter name=\"pattern\">circular</parameter>"
"        <parameter name=\"vector1.count\">6</parameter>"
"        <parameter name=\"vector2.count\">5</parameter>"
"        <parameter name=\"vector3.count\">5</parameter>"
"        <parameter name=\"circular.rotate\">1.0</parameter>"
"        <parameter name=\"circular.radius\">1.0</parameter>"
"        <parameter name=\"circular.normal_step\">1</parameter>"
"        <parameter name=\"circular.radial_step\">1</parameter>"
"        <parameter name=\"vector1.x\">1</parameter>"
"        <parameter name=\"vector2.y\">1</parameter>"
"        <object name=\"Object\" type=\"static_body\">"
"            <parameter name=\"mass\">100</parameter>"
"        </object>"
"    </object>"
"</EVDS>",&object));
		ERROR_CHECK(EVDS_Object_Initialize(object,1));

		/// Check if original object becomes part of the modifiers children container
		EQUAL_TO(EVDS_System_GetObjectByName(system,"Object",0,&object), EVDS_OK);
		EQUAL_TO(object->parent,root);
		EQUAL_TO(object->initialized,1);

		/// Check some key objects that must be part of the container
		EQUAL_TO(EVDS_System_GetObjectByName(system,"Object",root,&object), EVDS_OK);
		EQUAL_TO(object->initialized,1);
		VECTOR_EQUAL_TO(&object->state.position, 0,0,0);

		/// Check if all objects are placed in their correct positions
		/// If NORMAL/vector1 is (1,0,0), the rotation will only be in X plane
		/// If DIRECTION/vector2 is (0,1,0), the rotated objects will be directed towards +Y axis
		/// This corresponds with clockwise rotation, if looking ALONG the +X axis. The first object
		/// is located at (0,0,0). The first few objects will be at positive Y, negative Z.
		for (i = 1; i < 5; i++) {
			char name[256] = { 0 };

			sprintf(name,"Object (%dx1x1)",i+1);
			EQUAL_TO(EVDS_System_GetObjectByName(system,name,root,&object), EVDS_OK);
			EQUAL_TO(object->initialized,1);
			VECTOR_EQUAL_TO(&object->state.position, 0, 1.0 - cos(EVDS_RAD(i*60.0)), -sin(EVDS_RAD(i*60.0)));

			sprintf(name,"Object (%dx5x1)",i+1);
			EQUAL_TO(EVDS_System_GetObjectByName(system,name,root,&object), EVDS_OK);
			EQUAL_TO(object->initialized,1);
			VECTOR_EQUAL_TO(&object->state.position, 0, 1.0 - 5*cos(EVDS_RAD(i*60.0)), -5*sin(EVDS_RAD(i*60.0)));

			sprintf(name,"Object (%dx1x5)",i+1);
			EQUAL_TO(EVDS_System_GetObjectByName(system,name,root,&object), EVDS_OK);
			EQUAL_TO(object->initialized,1);
			VECTOR_EQUAL_TO(&object->state.position, 4.0, 1.0 - cos(EVDS_RAD(i*60.0)), -sin(EVDS_RAD(i*60.0)));

			sprintf(name,"Object (%dx5x5)",i+1);
			EQUAL_TO(EVDS_System_GetObjectByName(system,name,root,&object), EVDS_OK);
			EQUAL_TO(object->initialized,1);
			VECTOR_EQUAL_TO(&object->state.position, 4.0, 1.0 - 5*cos(EVDS_RAD(i*60.0)), -5*sin(EVDS_RAD(i*60.0)));
		}

		/// Check attitude of the children objects
		for (i = 1; i < 5; i++) {
			EVDS_REAL x,y,z,tgt_x;
			char name[256] = { 0 };

			sprintf(name,"Object (%dx1x1)",i+1);
			EQUAL_TO(EVDS_System_GetObjectByName(system,name,root,&object), EVDS_OK);
			EQUAL_TO(object->initialized,1);
			
			//Get euler angles and check them
			EVDS_Quaternion_ToEuler(&object->state.orientation,object->state.orientation.coordinate_system,&x,&y,&z);

			tgt_x = 60.0*i; //Check against properly wrapped angle
			if (tgt_x > 180.0) tgt_x = tgt_x - 360.0;

			REAL_EQUAL_TO_EPS(EVDS_DEG(x),tgt_x,EVDS_EPSf);
			REAL_EQUAL_TO_EPS(EVDS_DEG(y),0,EVDS_EPSf);
			REAL_EQUAL_TO_EPS(EVDS_DEG(z),0,EVDS_EPSf);
		}
	} END_TEST




	START_TEST("Circular modifier test (consistency of quaternions with modifiers transformation)") {
		int i,j,k;

		/// This test merely checks what happens when "yaw" is specified for modifier
		/// (it must rotate all objects by 90 deg)
		ERROR_CHECK(EVDS_Object_LoadFromString(root,
"<EVDS version=\"31\">"
"    <object name=\"Modifier\" type=\"modifier\" yaw=\"90\">"
"        <parameter name=\"pattern\">circular</parameter>"
"        <parameter name=\"vector1.count\">6</parameter>"
"        <parameter name=\"vector2.count\">5</parameter>"
"        <parameter name=\"vector3.count\">5</parameter>"
"        <parameter name=\"circular.rotate\">1.0</parameter>"
"        <parameter name=\"circular.radius\">1.0</parameter>"
"        <parameter name=\"circular.normal_step\">1</parameter>"
"        <parameter name=\"circular.radial_step\">1</parameter>"
"        <parameter name=\"vector1.x\">1</parameter>"
"        <parameter name=\"vector2.y\">1</parameter>"
"        <object name=\"Object\" type=\"static_body\">"
"            <parameter name=\"mass\">100</parameter>"
"        </object>"
"    </object>"
"</EVDS>",&object));
		ERROR_CHECK(EVDS_Object_Initialize(object,1));

		/// Check if original object becomes part of the modifiers children container
		EQUAL_TO(EVDS_System_GetObjectByName(system,"Object",0,&object), EVDS_OK);
		EQUAL_TO(object->parent,root);
		EQUAL_TO(object->initialized,1);

		/// Check attitude of the children objects
		for (i = 1; i < 5; i++) {
			EVDS_REAL x,y,z,tgt_x;
			char name[256] = { 0 };

			sprintf(name,"Object (%dx1x1)",i+1);
			EQUAL_TO(EVDS_System_GetObjectByName(system,name,root,&object), EVDS_OK);
			EQUAL_TO(object->initialized,1);
			
			//Get euler angles and check them
			EVDS_Quaternion_ToEuler(&object->state.orientation,root,&x,&y,&z);

			tgt_x = 60.0*i; //Check against properly wrapped angle. Note 90 deg offset
			if (tgt_x > 180.0) tgt_x = tgt_x - 360.0;

			REAL_EQUAL_TO_EPS(EVDS_DEG(x),tgt_x,EVDS_EPSf);
			REAL_EQUAL_TO_EPS(EVDS_DEG(y),0,EVDS_EPSf);
			REAL_EQUAL_TO_EPS(EVDS_DEG(z),90.0,EVDS_EPSf);
		}
	} END_TEST


	START_TEST("Pattern modifier test") {
		/// This test verifies that pattern modifier creates copies of children according to
		/// predefined set of points.
	} END_TEST
}




void Test_EVDS_GIMBAL() {
	START_TEST("Parameterless gimbal platform") {
		/// Test gimbal platform with default parameters. Such platform will instantly
		/// respond to its commands.
		EVDS_REAL x,y,z;
		EVDS_OBJECT* platform;
		EVDS_VARIABLE* pitch_command;
		EVDS_VARIABLE* yaw_command;
		ERROR_CHECK(EVDS_Object_LoadFromString(root,
"<EVDS version=\"31\">"
"    <object name=\"Gimbal\" type=\"gimbal\">"
"        <parameter name=\"mass\">1000</parameter>"
"        <object name=\"Rocket nozzle\" type=\"static_body\">"
"            <parameter name=\"mass\">100</parameter>"
"        </object>"
"    </object>"
"</EVDS>",&object));
		ERROR_CHECK(EVDS_Object_Initialize(object,1));

		/// Check that platform exists, and mass of the platform equals to mass specified for gimbal object
		EQUAL_TO(EVDS_System_GetObjectByName(system,"Gimbal (Platform)",0,&platform), EVDS_OK);
		EQUAL_TO(EVDS_System_GetObjectByName(system,"Rocket nozzle",0,&object), EVDS_OK);
		EQUAL_TO(object->parent,platform);
		EQUAL_TO(platform->initialized,1);
		ERROR_CHECK(EVDS_Object_GetVariable(platform,"mass",&variable));
		ERROR_CHECK(EVDS_Variable_GetReal(variable,&real));
		REAL_EQUAL_TO(real,1000.0);

		/// Get variables corresponding to platform commands
		ERROR_CHECK(EVDS_System_GetObjectByName(system,"Gimbal",0,&object));
		ERROR_CHECK(EVDS_Object_GetVariable(object,"pitch.command",&pitch_command));
		ERROR_CHECK(EVDS_Object_GetVariable(object,"yaw.command",&yaw_command));

		/// Get nozzle and make sure its initialized
		EQUAL_TO(EVDS_System_GetObjectByName(system,"Rocket nozzle",platform,&object), EVDS_OK);
		EQUAL_TO(object->initialized,1);

		/// Check response to commands. Reset gimbal to stationary position
		ERROR_CHECK(EVDS_Object_Solve(root,0.0)); //Initialize state
		EVDS_Quaternion_ToEuler(&object->state.orientation,root,&x,&y,&z);
		EQUAL_TO(x,EVDS_RAD(0.0));
		EQUAL_TO(y,EVDS_RAD(0.0));
		EQUAL_TO(z,EVDS_RAD(0.0));

		/// Check command response on pitch (must be instant)
		ERROR_CHECK(EVDS_Variable_SetReal(pitch_command,90.0));
		ERROR_CHECK(EVDS_Variable_SetReal(yaw_command,0.0));
		ERROR_CHECK(EVDS_Object_Solve(root,0.0));
		EVDS_Quaternion_ToEuler(&object->state.orientation,root,&x,&y,&z);
		REAL_EQUAL_TO(x,EVDS_RAD(0.0));
		REAL_EQUAL_TO(y,EVDS_RAD(90.0));
		REAL_EQUAL_TO(z,EVDS_RAD(0.0));

		ERROR_CHECK(EVDS_Variable_SetReal(pitch_command,-90.0));
		ERROR_CHECK(EVDS_Variable_SetReal(yaw_command,0.0));
		ERROR_CHECK(EVDS_Object_Solve(root,0.0));
		EVDS_Quaternion_ToEuler(&object->state.orientation,root,&x,&y,&z);
		REAL_EQUAL_TO(x,EVDS_RAD(0.0));
		REAL_EQUAL_TO(y,EVDS_RAD(-90.0));
		REAL_EQUAL_TO(z,EVDS_RAD(0.0));

		/// Check command response on yaw (must be instant)
		ERROR_CHECK(EVDS_Variable_SetReal(pitch_command,0.0));
		ERROR_CHECK(EVDS_Variable_SetReal(yaw_command,90.0));
		ERROR_CHECK(EVDS_Object_Solve(root,0.0));
		EVDS_Quaternion_ToEuler(&object->state.orientation,root,&x,&y,&z);
		REAL_EQUAL_TO(x,EVDS_RAD(0.0));
		REAL_EQUAL_TO(y,EVDS_RAD(0.0));
		REAL_EQUAL_TO(z,EVDS_RAD(90.0));

		ERROR_CHECK(EVDS_Variable_SetReal(pitch_command,0.0));
		ERROR_CHECK(EVDS_Variable_SetReal(yaw_command,-90.0));
		ERROR_CHECK(EVDS_Object_Solve(root,0.0));
		EVDS_Quaternion_ToEuler(&object->state.orientation,root,&x,&y,&z);
		REAL_EQUAL_TO(x,EVDS_RAD(0.0));
		REAL_EQUAL_TO(y,EVDS_RAD(0.0));
		REAL_EQUAL_TO(z,EVDS_RAD(-90.0));

		/// Check command response on mixed channels (must be instant)
		ERROR_CHECK(EVDS_Variable_SetReal(pitch_command,90.0));
		ERROR_CHECK(EVDS_Variable_SetReal(yaw_command,90.0));
		ERROR_CHECK(EVDS_Object_Solve(root,0.0));
		EVDS_Quaternion_ToEuler(&object->state.orientation,root,&x,&y,&z);
		REAL_EQUAL_TO(x,EVDS_RAD(0.0));
		REAL_EQUAL_TO(y,EVDS_RAD(90.0));
		REAL_EQUAL_TO(z,EVDS_RAD(90.0));

		ERROR_CHECK(EVDS_Variable_SetReal(pitch_command,-90.0));
		ERROR_CHECK(EVDS_Variable_SetReal(yaw_command,-90.0));
		ERROR_CHECK(EVDS_Object_Solve(root,0.0));
		EVDS_Quaternion_ToEuler(&object->state.orientation,root,&x,&y,&z);
		REAL_EQUAL_TO(x,EVDS_RAD(0.0));
		REAL_EQUAL_TO(y,EVDS_RAD(-90.0));
		REAL_EQUAL_TO(z,EVDS_RAD(-90.0));
	} END_TEST


	START_TEST("Gimbal platform behavior") {
		/// Test gimbal platform behavior. This test verifies that finite discrete states
		/// for platform deflection, min & max limits, platform movement rates all work.
		EVDS_REAL t,x,y,z;
		EVDS_OBJECT* platform;
		EVDS_VARIABLE* pitch_command;
		EVDS_VARIABLE* yaw_command;
		EVDS_VARIABLE* pitch_current;
		EVDS_VARIABLE* yaw_current;
		ERROR_CHECK(EVDS_Object_LoadFromString(root,
"<EVDS version=\"31\">"
"    <object name=\"Gimbal\" type=\"gimbal\">"
"        <parameter name=\"mass\">1000</parameter>"
"        <parameter name=\"pitch.min\">-10</parameter>"
"        <parameter name=\"pitch.max\">10</parameter>"
"        <parameter name=\"pitch.rate\">10</parameter>"
"        <parameter name=\"pitch.bits\">8</parameter>"
"        <parameter name=\"yaw.min\">-10</parameter>"
"        <parameter name=\"yaw.max\">10</parameter>"
"        <parameter name=\"yaw.rate\">20</parameter>"
"        <parameter name=\"yaw.bits\">16</parameter>"
"        <object name=\"Rocket nozzle\" type=\"static_body\">"
"            <parameter name=\"mass\">100</parameter>"
"        </object>"
"    </object>"
"</EVDS>",&object));
		ERROR_CHECK(EVDS_Object_Initialize(object,1));

		/// Setup all for testing
		ERROR_CHECK(EVDS_System_GetObjectByName(system,"Gimbal (Platform)",0,&platform));
		ERROR_CHECK(EVDS_System_GetObjectByName(system,"Gimbal",0,&object));
		ERROR_CHECK(EVDS_Object_GetVariable(object,"pitch.command",&pitch_command));
		ERROR_CHECK(EVDS_Object_GetVariable(object,"yaw.command",&yaw_command));
		ERROR_CHECK(EVDS_Object_GetVariable(object,"pitch.current",&pitch_current));
		ERROR_CHECK(EVDS_Object_GetVariable(object,"yaw.current",&yaw_current));
		ERROR_CHECK(EVDS_System_GetObjectByName(system,"Rocket nozzle",0,&object));

		/// Gimbal stationary (default command)
		ERROR_CHECK(EVDS_Object_Solve(root,0.0)); //Initialize state
		EVDS_Quaternion_ToEuler(&object->state.orientation,root,&x,&y,&z);
		EQUAL_TO(x,EVDS_RAD(0.0));
		EQUAL_TO(y,EVDS_RAD(0.0));
		EQUAL_TO(z,EVDS_RAD(0.0));

		/// Check response to commands on pitch
		/// This test verifies that gimbal moves with predefined rate on pitch channel
		ERROR_CHECK(EVDS_Variable_SetReal(pitch_command,90.0));
		ERROR_CHECK(EVDS_Variable_SetReal(yaw_command,0.0));
			ERROR_CHECK(EVDS_Object_Solve(root,0.1)); //0.1
			ERROR_CHECK(EVDS_Object_Solve(root,0.1));
			ERROR_CHECK(EVDS_Object_Solve(root,0.1));
			ERROR_CHECK(EVDS_Object_Solve(root,0.1));
			ERROR_CHECK(EVDS_Object_Solve(root,0.1)); //0.5 seconds
		EVDS_Quaternion_ToEuler(&object->state.orientation,root,&x,&y,&z);
		REAL_EQUAL_TO(y,EVDS_RAD(5.0)); //Gimbal must be half-way down
		REAL_EQUAL_TO(z,EVDS_RAD(0.0));
			ERROR_CHECK(EVDS_Object_Solve(root,0.1)); //0.6
			ERROR_CHECK(EVDS_Object_Solve(root,0.1));
			ERROR_CHECK(EVDS_Object_Solve(root,0.1));
			ERROR_CHECK(EVDS_Object_Solve(root,0.1));
			ERROR_CHECK(EVDS_Object_Solve(root,0.1)); //1.0 seconds
		EVDS_Quaternion_ToEuler(&object->state.orientation,root,&x,&y,&z);
		REAL_EQUAL_TO(y,EVDS_RAD(10.0)); //Gimbal must be fully down
		REAL_EQUAL_TO(z,EVDS_RAD(0.0));


		//Reset command (also test very long delta-times)
		ERROR_CHECK(EVDS_Variable_SetReal(pitch_command,0.0));
		ERROR_CHECK(EVDS_Variable_SetReal(yaw_command,0.0));
			ERROR_CHECK(EVDS_Object_Solve(root,100.0));
		EVDS_Quaternion_ToEuler(&object->state.orientation,root,&x,&y,&z);
		REAL_EQUAL_TO(y,EVDS_RAD(0.0));
		REAL_EQUAL_TO(z,EVDS_RAD(0.0));


		/// Check response to commands on yaw
		/// This test verifies that gimbal moves with predefined rate on yaw channel
		ERROR_CHECK(EVDS_Variable_SetReal(pitch_command,0.0));
		ERROR_CHECK(EVDS_Variable_SetReal(yaw_command,90.0));
			ERROR_CHECK(EVDS_Object_Solve(root,0.1)); //0.1
			ERROR_CHECK(EVDS_Object_Solve(root,0.1));
			ERROR_CHECK(EVDS_Object_Solve(root,0.1));
			ERROR_CHECK(EVDS_Object_Solve(root,0.1));
			ERROR_CHECK(EVDS_Object_Solve(root,0.1)); //0.5 seconds
		EVDS_Quaternion_ToEuler(&object->state.orientation,root,&x,&y,&z);
		REAL_EQUAL_TO(y,EVDS_RAD(0.0));
		REAL_EQUAL_TO(z,EVDS_RAD(10.0)); //Gimbal must be fully right


		/// Reset command
		ERROR_CHECK(EVDS_Variable_SetReal(pitch_command,0.0));
		ERROR_CHECK(EVDS_Variable_SetReal(yaw_command,0.0));
			ERROR_CHECK(EVDS_Object_Solve(root,100.0));
		EVDS_Quaternion_ToEuler(&object->state.orientation,root,&x,&y,&z);
		REAL_EQUAL_TO(y,EVDS_RAD(0.0));
		REAL_EQUAL_TO(z,EVDS_RAD(0.0));


		/// Check precision in bits
		/// Out of 8 bits of precision only 255 different values are used.
		/// This allows both max and min values, as well as midpoint value to be properly represented
		ERROR_CHECK(EVDS_Variable_SetReal(pitch_command,60*20.0/255.0 + 0.4*20.0/255.0));
		ERROR_CHECK(EVDS_Variable_SetReal(yaw_command,0.0));
			ERROR_CHECK(EVDS_Object_Solve(root,0.1)); //0.1
			ERROR_CHECK(EVDS_Object_Solve(root,0.1));
			ERROR_CHECK(EVDS_Object_Solve(root,0.1));
			ERROR_CHECK(EVDS_Object_Solve(root,0.1));
			ERROR_CHECK(EVDS_Object_Solve(root,0.1)); //Must take no longer than 0.5 seconds
		EVDS_Quaternion_ToEuler(&object->state.orientation,root,&x,&y,&z);
		REAL_EQUAL_TO(y,EVDS_RAD(60*20.0/255.0)); //Actual angle snaps because of precision limits
		REAL_EQUAL_TO(z,EVDS_RAD(0.0));

		/// Check both rounding up and down behavior (corresponds to internal transformation
		/// of raw commanded value into deflection angle assuming finite bits precision).
		ERROR_CHECK(EVDS_Variable_SetReal(pitch_command,60*20.0/255.0 - 0.4*20.0/255.0));
		ERROR_CHECK(EVDS_Variable_SetReal(yaw_command,0.0));
			ERROR_CHECK(EVDS_Object_Solve(root,0.1)); //0.1
		EVDS_Quaternion_ToEuler(&object->state.orientation,root,&x,&y,&z);
		REAL_EQUAL_TO(y,EVDS_RAD(60*20.0/255.0));
		REAL_EQUAL_TO(z,EVDS_RAD(0.0));



		/// Reset command
		ERROR_CHECK(EVDS_Variable_SetReal(pitch_command,0.0));
		ERROR_CHECK(EVDS_Variable_SetReal(yaw_command,0.0));
			ERROR_CHECK(EVDS_Object_Solve(root,100.0));
		EVDS_Quaternion_ToEuler(&object->state.orientation,root,&x,&y,&z);
		REAL_EQUAL_TO(y,EVDS_RAD(0.0));
		REAL_EQUAL_TO(z,EVDS_RAD(0.0));



		/// Run same checks with yaw (16 bits precision)
		ERROR_CHECK(EVDS_Variable_SetReal(pitch_command,0.0));
		ERROR_CHECK(EVDS_Variable_SetReal(yaw_command,10000*20.0/65535.0 + 0.4*20.0/65535.0));
			ERROR_CHECK(EVDS_Object_Solve(root,0.1)); //0.1
			ERROR_CHECK(EVDS_Object_Solve(root,0.1));
			ERROR_CHECK(EVDS_Object_Solve(root,0.05)); //Must take no longer than 0.25 seconds
		EVDS_Quaternion_ToEuler(&object->state.orientation,root,&x,&y,&z);
		REAL_EQUAL_TO(y,EVDS_RAD(0.0));
		REAL_EQUAL_TO(z,EVDS_RAD(10000*20.0/65535.0));

		/// Check both rounding up and down behavior (corresponds to internal transformation
		/// of raw commanded value into deflection angle assuming finite bits precision).
		ERROR_CHECK(EVDS_Variable_SetReal(pitch_command,0.0));
		ERROR_CHECK(EVDS_Variable_SetReal(yaw_command,10000*20.0/65535.0 - 0.4*20.0/65535.0));
			ERROR_CHECK(EVDS_Object_Solve(root,0.1));
		EVDS_Quaternion_ToEuler(&object->state.orientation,root,&x,&y,&z);
		REAL_EQUAL_TO(y,EVDS_RAD(0.0));
		REAL_EQUAL_TO(z,EVDS_RAD(10000*20.0/65535.0));
	} END_TEST
}




void Test_EVDS_ROCKET_ENGINE() {
	START_TEST("Rocket engine (nozzle parameters)") {
		////////////////////////////////////////////////////////////////////////
		ERROR_CHECK(EVDS_Object_LoadFromString(root,
"<EVDS version=\"31\">"
"    <object name=\"Rocket engine\" type=\"rocket_engine\">"
"        <parameter name=\"mass\">1000</parameter>"
"        <parameter name=\"nozzle.exit_radius\">2.0</parameter>"
"    </object>"
"</EVDS>",&object));
		ERROR_CHECK(EVDS_Object_Initialize(object,1));
		ERROR_CHECK(EVDS_Object_GetRealVariable(object,"nozzle.exit_area",&real,&variable));
		REAL_EQUAL_TO(real,EVDS_PI*4.0);

		////////////////////////////////////////////////////////////////////////
		ERROR_CHECK(EVDS_Object_LoadFromString(root,
"<EVDS version=\"31\">"
"    <object name=\"Rocket engine\" type=\"rocket_engine\">"
"        <parameter name=\"mass\">1000</parameter>"
"        <parameter name=\"nozzle.exit_area\">3.1415926</parameter>"
"    </object>"
"</EVDS>",&object));
		ERROR_CHECK(EVDS_Object_Initialize(object,1));
		ERROR_CHECK(EVDS_Object_GetRealVariable(object,"nozzle.exit_radius",&real,&variable));
		REAL_EQUAL_TO_EPS(real,1.0,EVDS_EPSf);
	} END_TEST


	START_TEST("Rocket engine (mass flow parameters)") {
		////////////////////////////////////////////////////////////////////////
		ERROR_CHECK(EVDS_Object_LoadFromString(root,
"<EVDS version=\"31\">"
"    <object name=\"Rocket engine\" type=\"rocket_engine\">"
"        <parameter name=\"mass\">1000</parameter>"
"        <parameter name=\"vacuum.isp\">200.0</parameter>"
"        <parameter name=\"vacuum.thrust\">200.0</parameter>"
"        <parameter name=\"atmospheric.isp\">100.0</parameter>"
"        <parameter name=\"atmospheric.thrust\">200.0</parameter>"
"        <parameter name=\"combustion.of_ratio\">3.0</parameter>"
"    </object>"
"</EVDS>",&object));
		ERROR_CHECK(EVDS_Object_Initialize(object,1));
		ERROR_CHECK(EVDS_Object_GetRealVariable(object,"vacuum.mass_flow",&real,&variable));
		REAL_EQUAL_TO(real,1.0/EVDS_G0);
		ERROR_CHECK(EVDS_Object_GetRealVariable(object,"vacuum.fuel_flow",&real,&variable));
		REAL_EQUAL_TO(real,0.25*1.0/EVDS_G0);
		ERROR_CHECK(EVDS_Object_GetRealVariable(object,"vacuum.oxidizer_flow",&real,&variable));
		REAL_EQUAL_TO(real,0.75*1.0/EVDS_G0);

		ERROR_CHECK(EVDS_Object_GetRealVariable(object,"atmospheric.mass_flow",&real,&variable));
		REAL_EQUAL_TO(real,2.0/EVDS_G0);
		ERROR_CHECK(EVDS_Object_GetRealVariable(object,"atmospheric.fuel_flow",&real,&variable));
		REAL_EQUAL_TO(real,0.25*2.0/EVDS_G0);
		ERROR_CHECK(EVDS_Object_GetRealVariable(object,"atmospheric.oxidizer_flow",&real,&variable));
		REAL_EQUAL_TO(real,0.75*2.0/EVDS_G0);
	} END_TEST


	START_TEST("Rocket engine (Isp/Ve parameters)") {
		////////////////////////////////////////////////////////////////////////
		ERROR_CHECK(EVDS_Object_LoadFromString(root,
"<EVDS version=\"31\">"
"    <object name=\"Rocket engine\" type=\"rocket_engine\">"
"        <parameter name=\"mass\">1000</parameter>"
"        <parameter name=\"vacuum.isp\">200.0</parameter>"
"        <parameter name=\"atmospheric.isp\">100.0</parameter>"
"    </object>"
"</EVDS>",&object));
		ERROR_CHECK(EVDS_Object_Initialize(object,1));
		ERROR_CHECK(EVDS_Object_GetRealVariable(object,"vacuum.exhaust_velocity",&real,&variable));
		REAL_EQUAL_TO(real,200.0*EVDS_G0);
		ERROR_CHECK(EVDS_Object_GetRealVariable(object,"atmospheric.exhaust_velocity",&real,&variable));
		REAL_EQUAL_TO(real,100.0*EVDS_G0);

		////////////////////////////////////////////////////////////////////////
		ERROR_CHECK(EVDS_Object_LoadFromString(root,
"<EVDS version=\"31\">"
"    <object name=\"Rocket engine\" type=\"rocket_engine\">"
"        <parameter name=\"mass\">1000</parameter>"
"        <parameter name=\"vacuum.exhaust_velocity\">980.7</parameter>"
"        <parameter name=\"atmospheric.exhaust_velocity\">1961.4</parameter>"
"    </object>"
"</EVDS>",&object));
		ERROR_CHECK(EVDS_Object_Initialize(object,1));
		ERROR_CHECK(EVDS_Object_GetRealVariable(object,"vacuum.isp",&real,&variable));
		REAL_EQUAL_TO_EPS(real,100.0,1e-2);
		ERROR_CHECK(EVDS_Object_GetRealVariable(object,"atmospheric.isp",&real,&variable));
		REAL_EQUAL_TO_EPS(real,200.0,1e-2);
	} END_TEST


	START_TEST("Rocket engine (thrust parameters)") {
		////////////////////////////////////////////////////////////////////////
		ERROR_CHECK(EVDS_Object_LoadFromString(root,
"<EVDS version=\"31\">"
"    <object name=\"Rocket engine\" type=\"rocket_engine\">"
"        <parameter name=\"mass\">1000</parameter>"
"        <parameter name=\"vacuum.mass_flow\">10.0</parameter>"
"        <parameter name=\"vacuum.thrust\">100.0</parameter>"
"        <parameter name=\"atmospheric.mass_flow\">10.0</parameter>"
"        <parameter name=\"atmospheric.thrust\">50.0</parameter>"
"    </object>"
"</EVDS>",&object));
		ERROR_CHECK(EVDS_Object_Initialize(object,1));
		ERROR_CHECK(EVDS_Object_GetRealVariable(object,"vacuum.exhaust_velocity",&real,&variable));
		REAL_EQUAL_TO(real,10.0);
		ERROR_CHECK(EVDS_Object_GetRealVariable(object,"atmospheric.exhaust_velocity",&real,&variable));
		REAL_EQUAL_TO(real,5.0);
	} END_TEST


	START_TEST("Rocket engine (fallback/hacky parameters)") {
		////////////////////////////////////////////////////////////////////////
		ERROR_CHECK(EVDS_Object_LoadFromString(root,
"<EVDS version=\"31\">"
"    <object name=\"Rocket engine\" type=\"rocket_engine\">"
"        <parameter name=\"mass\">1000</parameter>"
"        <parameter name=\"vacuum.thrust\">10.0</parameter>"
"        <parameter name=\"vacuum.isp\">100.0</parameter>"
"    </object>"
"</EVDS>",&object));
		ERROR_CHECK(EVDS_Object_Initialize(object,1));
		ERROR_CHECK(EVDS_Object_GetRealVariable(object,"atmospheric.thrust",&real,&variable));
		REAL_EQUAL_TO(real,9.0);
		ERROR_CHECK(EVDS_Object_GetRealVariable(object,"atmospheric.isp",&real,&variable));
		REAL_EQUAL_TO(real,90.0);

		////////////////////////////////////////////////////////////////////////
		ERROR_CHECK(EVDS_Object_LoadFromString(root,
"<EVDS version=\"31\">"
"    <object name=\"Rocket engine\" type=\"rocket_engine\">"
"        <parameter name=\"mass\">1000</parameter>"
"        <parameter name=\"atmospheric.thrust\">9.0</parameter>"
"        <parameter name=\"atmospheric.isp\">90.0</parameter>"
"    </object>"
"</EVDS>",&object));
		ERROR_CHECK(EVDS_Object_Initialize(object,1));
		ERROR_CHECK(EVDS_Object_GetRealVariable(object,"vacuum.thrust",&real,&variable));
		REAL_EQUAL_TO(real,10.0);
		ERROR_CHECK(EVDS_Object_GetRealVariable(object,"vacuum.isp",&real,&variable));
		REAL_EQUAL_TO(real,100.0);
	} END_TEST


	START_TEST("Rocket engine (mass flow parameters for monopropellant engine)") {
		////////////////////////////////////////////////////////////////////////
		ERROR_CHECK(EVDS_Object_LoadFromString(root,
"<EVDS version=\"31\">"
"    <object name=\"Rocket engine\" type=\"rocket_engine\">"
"        <parameter name=\"mass\">1000</parameter>"
"        <parameter name=\"vacuum.isp\">200.0</parameter>"
"        <parameter name=\"vacuum.thrust\">200.0</parameter>"
"        <parameter name=\"atmospheric.isp\">100.0</parameter>"
"        <parameter name=\"atmospheric.thrust\">200.0</parameter>"
"    </object>"
"</EVDS>",&object));
		ERROR_CHECK(EVDS_Object_Initialize(object,1));
		ERROR_CHECK(EVDS_Object_GetRealVariable(object,"vacuum.mass_flow",&real,&variable));
		REAL_EQUAL_TO(real,1.0/EVDS_G0);
		ERROR_CHECK(EVDS_Object_GetRealVariable(object,"vacuum.fuel_flow",&real,&variable));
		REAL_EQUAL_TO(real,1.0/EVDS_G0);

		ERROR_CHECK(EVDS_Object_GetRealVariable(object,"atmospheric.mass_flow",&real,&variable));
		REAL_EQUAL_TO(real,2.0/EVDS_G0);
		ERROR_CHECK(EVDS_Object_GetRealVariable(object,"atmospheric.fuel_flow",&real,&variable));
		REAL_EQUAL_TO(real,2.0/EVDS_G0);
	} END_TEST



	START_TEST("Rocket engine (basic tests)") {
		////////////////////////////////////////////////////////////////////////
		EVDS_OBJECT* engine;
		EVDS_VARIABLE* command_throttle;
		ERROR_CHECK(EVDS_Object_LoadFromString(root,
"<EVDS version=\"34\">"
"    <object name=\"Vessel\" type=\"vessel\">"
"        <object name=\"Fuel\" type=\"fuel_tank\">"
"            <parameter name=\"fuel.type\">O2</parameter>"
"            <parameter name=\"fuel.mass\">4000</parameter>"
"        </object>"
"        <object name=\"Fuel\" type=\"fuel_tank\">"
"            <parameter name=\"fuel.type\">H2</parameter>"
"            <parameter name=\"fuel.mass\">1000</parameter>"
"        </object>"
"        <object name=\"Rocket engine\" type=\"rocket_engine\">"
"            <parameter name=\"mass\">1000</parameter>"
"            <parameter name=\"vacuum.isp\">400.0</parameter>"
"            <parameter name=\"vacuum.thrust\">100.0</parameter>"
"        </object>"
"    </object>"
"</EVDS>",&object));
		ERROR_CHECK(EVDS_Object_Initialize(object,1));
		ERROR_CHECK(EVDS_System_GetObjectByName(system,"Rocket engine",0,&engine));
		ERROR_CHECK(EVDS_Object_GetVariable(engine,"command.throttle",&command_throttle));

		//Check automatically detected fuel type
		ERROR_CHECK(EVDS_Object_GetVariable(engine,"combustion.fuel",&variable));
		ERROR_CHECK(EVDS_Variable_GetString(variable,string,8192,0));
		STRING_EQUAL_TO(string,"H2");

		ERROR_CHECK(EVDS_Object_GetVariable(engine,"combustion.oxidizer",&variable));
		ERROR_CHECK(EVDS_Variable_GetString(variable,string,8192,0));
		STRING_EQUAL_TO(string,"O2");

		//Automatically detected O:F ratio
		ERROR_CHECK(EVDS_Object_GetRealVariable(engine,"combustion.of_ratio",&real,&variable));
		REAL_EQUAL_TO(real,4.0);


		//Command throttle to 100% and check current parameters
		ERROR_CHECK(EVDS_Variable_SetReal(command_throttle,1.0));
		ERROR_CHECK(EVDS_Object_Solve(object,0.0));

		ERROR_CHECK(EVDS_Object_GetRealVariable(engine,"current.thrust",&real,&variable));
		REAL_EQUAL_TO(real,100.0);
		ERROR_CHECK(EVDS_Object_GetRealVariable(engine,"current.isp",&real,&variable));
		REAL_EQUAL_TO(real,400.0);
		ERROR_CHECK(EVDS_Object_GetRealVariable(engine,"current.throttle",&real,&variable));
		REAL_EQUAL_TO(real,1.0);

		ERROR_CHECK(EVDS_Object_GetRealVariable(engine,"current.mass_flow",&real,&variable));
		REAL_EQUAL_TO(real,100.0/(EVDS_G0*400.0));
		ERROR_CHECK(EVDS_Object_GetRealVariable(engine,"current.fuel_flow",&real,&variable));
		REAL_EQUAL_TO(real,0.20*100.0/(EVDS_G0*400.0));
		ERROR_CHECK(EVDS_Object_GetRealVariable(engine,"current.oxidizer_flow",&real,&variable));
		REAL_EQUAL_TO(real,0.80*100.0/(EVDS_G0*400.0));


		//Command throttle to 50% and check current parameters
		ERROR_CHECK(EVDS_Variable_SetReal(command_throttle,0.5));
		ERROR_CHECK(EVDS_Object_Solve(object,0.0));

		ERROR_CHECK(EVDS_Object_GetRealVariable(engine,"current.thrust",&real,&variable));
		REAL_EQUAL_TO(real,50.0);
		ERROR_CHECK(EVDS_Object_GetRealVariable(engine,"current.isp",&real,&variable));
		REAL_EQUAL_TO(real,400.0);
		ERROR_CHECK(EVDS_Object_GetRealVariable(engine,"current.throttle",&real,&variable));
		REAL_EQUAL_TO(real,0.5);

		ERROR_CHECK(EVDS_Object_GetRealVariable(engine,"current.mass_flow",&real,&variable));
		REAL_EQUAL_TO(real,50.0/(EVDS_G0*400.0));
		ERROR_CHECK(EVDS_Object_GetRealVariable(engine,"current.fuel_flow",&real,&variable));
		REAL_EQUAL_TO(real,0.20*50.0/(EVDS_G0*400.0));
		ERROR_CHECK(EVDS_Object_GetRealVariable(engine,"current.oxidizer_flow",&real,&variable));
		REAL_EQUAL_TO(real,0.80*50.0/(EVDS_G0*400.0));


		//Check actual returned thrust
		ERROR_CHECK(EVDS_Object_Integrate(object,0.0,0,&derivative));
		VECTOR_EQUAL_TO(&derivative.force,-50.0,0.0,0.0); //Half thrust

		EVDS_Vector_GetPositionVector(&derivative.force,&vector); //Must be located in reference point
		EQUAL_TO(vector.coordinate_system,object);
		VECTOR_EQUAL_TO(&vector,0.0,0.0,0.0);
	} END_TEST
		


	START_TEST("Rocket engine (default transient behavior tests)") {
		////////////////////////////////////////////////////////////////////////
		EVDS_OBJECT* engine;
		EVDS_VARIABLE* command_throttle;
		ERROR_CHECK(EVDS_Object_LoadFromString(root,
"<EVDS version=\"34\">"
"    <object name=\"Vessel\" type=\"vessel\">"
"        <object name=\"Oxidizer\" type=\"fuel_tank\">"
"            <parameter name=\"fuel.type\">O2</parameter>"
"            <parameter name=\"fuel.mass\">4000</parameter>"
"        </object>"
"        <object name=\"Fuel\" type=\"fuel_tank\">"
"            <parameter name=\"fuel.type\">H2</parameter>"
"            <parameter name=\"fuel.mass\">1000</parameter>"
"        </object>"
"        <object name=\"Rocket engine\" type=\"rocket_engine\">"
"            <parameter name=\"mass\">1000</parameter>"
"            <parameter name=\"control.throttle_speed\">0.25</parameter>"
"            <parameter name=\"control.min_throttle\">0.5</parameter>"
"            <parameter name=\"control.max_throttle\">1.0</parameter>"
"            <parameter name=\"control.startup_time\">2.0</parameter>"
"            <parameter name=\"control.shutdown_time\">0.5</parameter>"
"            <parameter name=\"vacuum.isp\">400.0</parameter>"
"            <parameter name=\"vacuum.thrust\">100000.0</parameter>"
"        </object>"
"    </object>"
"</EVDS>",&object));
		ERROR_CHECK(EVDS_Object_Initialize(object,1));
		ERROR_CHECK(EVDS_System_GetObjectByName(system,"Rocket engine",0,&engine));
		ERROR_CHECK(EVDS_Object_GetVariable(engine,"command.throttle",&command_throttle));

		//Reset throttle
		ERROR_CHECK(EVDS_Variable_SetReal(command_throttle,0.0));
		ERROR_CHECK(EVDS_Object_Solve(object,0.0));

#define SIMULATE_FOR_TIME(T) \
		{ \
			double time; \
			for (time = 0; time < T; time += 0.01) ERROR_CHECK(EVDS_Object_Solve(engine,0.01)); \
		}

		//Idle engine for 1 second
		SIMULATE_FOR_TIME(1.0);
		ERROR_CHECK(EVDS_Object_GetRealVariable(engine,"current.throttle",&real,&variable));
		//Check actual throttle
		ERROR_CHECK(EVDS_Object_GetRealVariable(engine,"current.throttle",&real,&variable));
		REAL_EQUAL_TO(real,0.0);


		//Start engine up to intermediate throttle level
		ERROR_CHECK(EVDS_Variable_SetReal(command_throttle,0.5));

		//Check throttle 1.0 seconds in (half the startup time)
		SIMULATE_FOR_TIME(1.0);
		ERROR_CHECK(EVDS_Object_GetRealVariable(engine,"current.throttle",&real,&variable));
		REAL_EQUAL_TO_EPS(real,0.77*0.50,0.01); //Thrust must be 77% +- 1%
		//Check throttle 2.0 seconds in (at startup time)
		SIMULATE_FOR_TIME(1.0);
		ERROR_CHECK(EVDS_Object_GetRealVariable(engine,"current.throttle",&real,&variable));
		REAL_EQUAL_TO_EPS(real,0.95*0.50,0.01); //Thrust must be 95% +- 1%

		//Run engine at low throttle for 5 seconds
		SIMULATE_FOR_TIME(5.0);
		ERROR_CHECK(EVDS_Object_GetRealVariable(engine,"current.throttle",&real,&variable));
		REAL_EQUAL_TO_EPS(real,1.00*0.50,0.001); //Thrust must be 100% +- 0.01%


		//Throttle up to 100%
		ERROR_CHECK(EVDS_Variable_SetReal(command_throttle,1.0));

		//Check transient (100% must be reached in 2.0 seconds)
		ERROR_CHECK(EVDS_Object_GetRealVariable(engine,"current.throttle",&real,&variable));
		REAL_EQUAL_TO_EPS(real,0.50,0.001);

		SIMULATE_FOR_TIME(2.0);
		ERROR_CHECK(EVDS_Object_GetRealVariable(engine,"current.throttle",&real,&variable));
		//REAL_EQUAL_TO_EPS(real,1.00,0.001);
		EQUAL_TO(real > 0.95,1); //Check for exponential and linear laws (if they change)

		//Shutdown engine
		ERROR_CHECK(EVDS_Variable_SetReal(command_throttle, 0.0));

		//Check throttle falloff
		SIMULATE_FOR_TIME(0.25);
		ERROR_CHECK(EVDS_Object_GetRealVariable(engine,"current.throttle",&real,&variable));
		//REAL_EQUAL_TO_EPS(real,0.23*1.00,0.01); //22% halfway into shutdown time
		SIMULATE_FOR_TIME(0.25);
		ERROR_CHECK(EVDS_Object_GetRealVariable(engine,"current.throttle",&real,&variable));
		REAL_EQUAL_TO_EPS(real,0.05*1.00,0.01); //5% at shutdown time


		//Idle for some time
		SIMULATE_FOR_TIME(5.0);
		ERROR_CHECK(EVDS_Object_GetRealVariable(engine,"current.throttle",&real,&variable));
		//Check actual throttle
		ERROR_CHECK(EVDS_Object_GetRealVariable(engine,"current.throttle",&real,&variable));
		REAL_EQUAL_TO_EPS(real,0.0,0.001);


		//Start engine up again
		ERROR_CHECK(EVDS_Variable_SetReal(command_throttle,0.50));
		//Check thrust mid-start
		SIMULATE_FOR_TIME(1.0);
		ERROR_CHECK(EVDS_Object_GetRealVariable(engine,"current.throttle",&real,&variable));
		REAL_EQUAL_TO_EPS(real,0.77*0.50,0.01); //Thrust must be 77% +- 1%

		//Shutdown mid-start
		ERROR_CHECK(EVDS_Variable_SetReal(command_throttle,0.00));
		SIMULATE_FOR_TIME(0.25);
		ERROR_CHECK(EVDS_Object_GetRealVariable(engine,"current.throttle",&real,&variable));
		REAL_EQUAL_TO_EPS(real,0.23*0.77*0.50,0.01); //22%, including startup transient and throttle


		//Idle for some time
		SIMULATE_FOR_TIME(5.0);
		ERROR_CHECK(EVDS_Object_GetRealVariable(engine,"current.throttle",&real,&variable));
		//Check actual throttle
		ERROR_CHECK(EVDS_Object_GetRealVariable(engine,"current.throttle",&real,&variable));
		REAL_EQUAL_TO_EPS(real,0.0,0.001);


		//Start engine up
		ERROR_CHECK(EVDS_Variable_SetReal(command_throttle,1.00));
		//Let engine start up and throttle up
		SIMULATE_FOR_TIME(10.0);

		//Check low throttling limit
		ERROR_CHECK(EVDS_Variable_SetReal(command_throttle,0.45));
		//Let engine change throttle and work for some time
		SIMULATE_FOR_TIME(5.0);
		ERROR_CHECK(EVDS_Object_GetRealVariable(engine,"current.throttle",&real,&variable));
		REAL_EQUAL_TO_EPS(real,0.50,0.01); //50% throttle

		//Check high throttling limit
		ERROR_CHECK(EVDS_Variable_SetReal(command_throttle,1.50));
		//Let engine change throttle and work for some time
		SIMULATE_FOR_TIME(5.0);
		ERROR_CHECK(EVDS_Object_GetRealVariable(engine,"current.throttle",&real,&variable));
		REAL_EQUAL_TO_EPS(real,1.00,0.01); //100% throttle
	} END_TEST




	START_TEST("Rocket engine (fuel consumption tests)") {
		////////////////////////////////////////////////////////////////////////
		EVDS_OBJECT* engine;
		EVDS_VARIABLE* command_throttle;
		ERROR_CHECK(EVDS_Object_LoadFromString(root,
"<EVDS version=\"34\">"
"    <object name=\"Vessel\" type=\"vessel\">"
"        <object name=\"Oxidizer\" type=\"fuel_tank\">"
"            <parameter name=\"fuel.type\">O2</parameter>"
"            <parameter name=\"fuel.mass\">400</parameter>"
"        </object>"
"        <object name=\"Fuel\" type=\"fuel_tank\">"
"            <parameter name=\"fuel.type\">H2</parameter>"
"            <parameter name=\"fuel.mass\">100</parameter>"
"        </object>"
"        <object name=\"Rocket engine\" type=\"rocket_engine\">"
"            <parameter name=\"mass\">1000</parameter>"
"            <parameter name=\"vacuum.exhaust_velocity\">1000.0</parameter>"
"            <parameter name=\"vacuum.thrust\">1000.0</parameter>"
"        </object>"
"    </object>"
"</EVDS>",&object));
		ERROR_CHECK(EVDS_Object_Initialize(object,1));
		ERROR_CHECK(EVDS_System_GetObjectByName(system,"Rocket engine",0,&engine));
		ERROR_CHECK(EVDS_Object_GetVariable(engine,"command.throttle",&command_throttle));

		//Reset throttle
		ERROR_CHECK(EVDS_Variable_SetReal(command_throttle,0.0));
		ERROR_CHECK(EVDS_Object_Solve(object,0.0));

		//Startup engine and run it for some time
		ERROR_CHECK(EVDS_Variable_SetReal(command_throttle,1.0));
		for (real = 0.0; real < 250.0; real += 1.0) {
			ERROR_CHECK(EVDS_Object_Solve(engine,1.0));
		}

		//Mass flow is 1.0 kg/sec. The propellants should be worth for 500.0 seconds of flight.
		//After 250 seconds, half of propellants must be remaining

		//Check remaining fuel
		ERROR_CHECK(EVDS_System_GetObjectByName(system,"Fuel",0,&object));
		ERROR_CHECK(EVDS_Object_GetRealVariable(object,"fuel.mass",&real,&variable));
		REAL_EQUAL_TO_EPS(real,50.0,EVDS_EPSf);

		//Check remaining oxidizer
		ERROR_CHECK(EVDS_System_GetObjectByName(system,"Oxidizer",0,&object));
		ERROR_CHECK(EVDS_Object_GetRealVariable(object,"fuel.mass",&real,&variable));
		REAL_EQUAL_TO_EPS(real,200.0,EVDS_EPSf);

		//Run engine for 300.0 more seconds (so it depletes fuel)
		for (real = 0.0; real < 300.0; real += 1.0) {
			ERROR_CHECK(EVDS_Object_Solve(engine,1.0));
		}

		//Check remaining fuel
		ERROR_CHECK(EVDS_System_GetObjectByName(system,"Fuel",0,&object));
		ERROR_CHECK(EVDS_Object_GetRealVariable(object,"fuel.mass",&real,&variable));
		REAL_EQUAL_TO(real,0.0);

		//Check remaining oxidizer
		ERROR_CHECK(EVDS_System_GetObjectByName(system,"Oxidizer",0,&object));
		ERROR_CHECK(EVDS_Object_GetRealVariable(object,"fuel.mass",&real,&variable));
		REAL_EQUAL_TO(real,0.0);
	} END_TEST
}




void Test_EVDS_WING() {
	START_TEST("Wing (general)") {
			ERROR_CHECK(EVDS_System_DatabaseFromString(system,
"<EVDS>"
"	<database name=\"airfoils\">"
"		<entry name=\"NACA-2412\" print=\"NACA 2412\">"
"			<parameter name=\"Cl\" interpolation=\"linear\">"
"			</parameter>"
"			<parameter name=\"Cd\" interpolation=\"linear\">"
"			</parameter>"
"			<parameter name=\"Cm\" interpolation=\"linear\">"
"			</parameter>"
"			<parameter name=\"geometry\" interpolation=\"linear\">"
"			</parameter>"
"		</entry>"
"	</database>"
"</EVDS>"));


		/*ERROR_CHECK(EVDS_Object_LoadFromString(root,
"<EVDS version=\"31\">"
"    <object name=\"Rocket engine\" type=\"rocket_engine\">"
"        <parameter name=\"mass\">1000</parameter>"
"        <parameter name=\"nozzle.exit_radius\">2.0</parameter>"
"    </object>"
"</EVDS>",&object));
		ERROR_CHECK(EVDS_Object_Initialize(object,1));
		ERROR_CHECK(EVDS_Object_GetRealVariable(object,"nozzle.exit_area",&real,&variable));
		REAL_EQUAL_TO(real,EVDS_PI*4.0);*/
	} END_TEST
}