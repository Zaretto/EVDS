#include "framework.h"

void Test_EVDS_FRAMES() {
	START_TEST("LVLH coordinates (general)") {
		/// These tests make sure direction vectors in LVLH coordinates correspond properly
		/// to direction vectors in inertial coordinates.
		EVDS_VECTOR x,y,z;
		EVDS_GEODETIC_COORDINATE geocoord = { 0 };
		EVDS_OBJECT* earth;
		EVDS_OBJECT* frame;

		ERROR_CHECK(EVDS_Object_LoadFromString(root,
"<EVDS version=\"31\">"
"	<object name=\"Earth\" type=\"planet\">"
"		<parameter name=\"gravity.mu\">398600440000000</parameter>"
"		<parameter name=\"geometry.radius\">6378145.0</parameter>" //Spherical planet
"	</object>"
"</EVDS>",&earth));
		ERROR_CHECK(EVDS_Object_Initialize(earth,1));

		/// Create coordinate frame
		ERROR_CHECK(EVDS_Object_Create(earth,&frame));

		/// Check normal vector
		EVDS_Geodetic_Set(&geocoord,earth,0,0,1000);
		EVDS_LVLH_GetStateVector(&frame->state,&geocoord);
		EVDS_Vector_Set(&vector,EVDS_VECTOR_DIRECTION,frame,0,0,1);
		EVDS_Vector_Convert(&vector,&vector,root);
		VECTOR_EQUAL_TO(&vector,1,0,0);

		EVDS_Geodetic_Set(&geocoord,earth,0,90,1000);
		EVDS_LVLH_GetStateVector(&frame->state,&geocoord);
		EVDS_Vector_Set(&vector,EVDS_VECTOR_DIRECTION,frame,0,0,1);
		EVDS_Vector_Convert(&vector,&vector,root);
		VECTOR_EQUAL_TO(&vector,0,1,0);

		EVDS_Geodetic_Set(&geocoord,earth,0,-180,1000);
		EVDS_LVLH_GetStateVector(&frame->state,&geocoord);
		EVDS_Vector_Set(&vector,EVDS_VECTOR_DIRECTION,frame,0,0,1);
		EVDS_Vector_Convert(&vector,&vector,root);
		VECTOR_EQUAL_TO(&vector,-1,0,0);

		EVDS_Geodetic_Set(&geocoord,earth,0,-90,1000);
		EVDS_LVLH_GetStateVector(&frame->state,&geocoord);
		EVDS_Vector_Set(&vector,EVDS_VECTOR_DIRECTION,frame,0,0,1);
		EVDS_Vector_Convert(&vector,&vector,root);
		VECTOR_EQUAL_TO(&vector,0,-1,0);


		EVDS_Geodetic_Set(&geocoord,earth,90,0,1000);
		EVDS_LVLH_GetStateVector(&frame->state,&geocoord);
		EVDS_Vector_Set(&vector,EVDS_VECTOR_DIRECTION,frame,0,0,1);
		EVDS_Vector_Convert(&vector,&vector,root);
		VECTOR_EQUAL_TO(&vector,0,0,1);

		EVDS_Geodetic_Set(&geocoord,earth,-90,0,1000);
		EVDS_LVLH_GetStateVector(&frame->state,&geocoord);
		EVDS_Vector_Set(&vector,EVDS_VECTOR_DIRECTION,frame,0,0,1);
		EVDS_Vector_Convert(&vector,&vector,root);
		VECTOR_EQUAL_TO(&vector,0,0,-1);


		EVDS_Geodetic_Set(&geocoord,earth,90,90,1000);
		EVDS_LVLH_GetStateVector(&frame->state,&geocoord);
		EVDS_Vector_Set(&vector,EVDS_VECTOR_DIRECTION,frame,0,0,1);
		EVDS_Vector_Convert(&vector,&vector,root);
		VECTOR_EQUAL_TO(&vector,0,0,1);

		EVDS_Geodetic_Set(&geocoord,earth,90,-180,1000);
		EVDS_LVLH_GetStateVector(&frame->state,&geocoord);
		EVDS_Vector_Set(&vector,EVDS_VECTOR_DIRECTION,frame,0,0,1);
		EVDS_Vector_Convert(&vector,&vector,root);
		VECTOR_EQUAL_TO(&vector,0,0,1);

		EVDS_Geodetic_Set(&geocoord,earth,90,-90,1000);
		EVDS_LVLH_GetStateVector(&frame->state,&geocoord);
		EVDS_Vector_Set(&vector,EVDS_VECTOR_DIRECTION,frame,0,0,1);
		EVDS_Vector_Convert(&vector,&vector,root);
		VECTOR_EQUAL_TO(&vector,0,0,1);


		/// Check coordinate systems in few node points
		EVDS_Geodetic_Set(&geocoord,earth,0,0,1000);
		EVDS_LVLH_GetStateVector(&frame->state,&geocoord);
		EVDS_Vector_Set(&x,EVDS_VECTOR_DIRECTION,frame,1,0,0);
		EVDS_Vector_Set(&y,EVDS_VECTOR_DIRECTION,frame,0,1,0);
		EVDS_Vector_Set(&z,EVDS_VECTOR_DIRECTION,frame,0,0,1);
		EVDS_Vector_Convert(&x,&x,root);
		EVDS_Vector_Convert(&y,&y,root);
		EVDS_Vector_Convert(&z,&z,root);
		VECTOR_EQUAL_TO(&x,0,0,-1);
		VECTOR_EQUAL_TO(&y,0,1,0);
		VECTOR_EQUAL_TO(&z,1,0,0);


		EVDS_Geodetic_Set(&geocoord,earth,0,90,1000);
		EVDS_LVLH_GetStateVector(&frame->state,&geocoord);
		EVDS_Vector_Set(&x,EVDS_VECTOR_DIRECTION,frame,1,0,0);
		EVDS_Vector_Set(&y,EVDS_VECTOR_DIRECTION,frame,0,1,0);
		EVDS_Vector_Set(&z,EVDS_VECTOR_DIRECTION,frame,0,0,1);
		EVDS_Vector_Convert(&x,&x,root);
		EVDS_Vector_Convert(&y,&y,root);
		EVDS_Vector_Convert(&z,&z,root);
		VECTOR_EQUAL_TO(&x,0,0,-1);
		VECTOR_EQUAL_TO(&y,-1,0,0);
		VECTOR_EQUAL_TO(&z,0,1,0);
	} END_TEST


	START_TEST("LVLH coordinates (quaternion conversion equivalence)") {
		/// This test makes sure that forward and backward conversion between
		/// LVLH quaternion and inertial quaternion is correct.
		EVDS_REAL lat,lon;
		EVDS_QUATERNION src;
		EVDS_GEODETIC_COORDINATE geocoord = { 0 };
		EVDS_OBJECT* earth;
		ERROR_CHECK(EVDS_Object_LoadFromString(root,
"<EVDS version=\"31\">"
"	<object name=\"Earth\" type=\"planet\">"
"		<parameter name=\"gravity.mu\">398600440000000</parameter>"
"		<parameter name=\"geometry.radius\">6378145.0</parameter>" //Spherical planet
"	</object>"
"</EVDS>",&earth));
		ERROR_CHECK(EVDS_Object_Initialize(earth,1));

		//Set coordinate frame
		for (lat = -90.0; lat <= 90.0; lat += 20.0) {
			for (lon = -180.0; lon < 180.0; lon += 20.0) {
				EVDS_Geodetic_Set(&geocoord,earth,lat,lon,1000);
				EVDS_Quaternion_FromEuler(&src,earth,EVDS_RAD(123),EVDS_RAD(456),EVDS_RAD(789));
				EVDS_LVLH_QuaternionFromLVLH(&quaternion,&src,&geocoord);
				EVDS_LVLH_QuaternionToLVLH(&quaternion,&quaternion,&geocoord);
				REAL_EQUAL_TO(src.q[0],quaternion.q[0]);
				REAL_EQUAL_TO(src.q[1],quaternion.q[1]);
				REAL_EQUAL_TO(src.q[2],quaternion.q[2]);
				REAL_EQUAL_TO(src.q[3],quaternion.q[3]);

				/*EVDS_LVLH_QuaternionToLVLH(&quaternion,&src,&geocoord);
				EVDS_LVLH_QuaternionFromLVLH(&quaternion,&quaternion,&geocoord);
				REAL_EQUAL_TO(src.q[0],quaternion.q[0]);
				REAL_EQUAL_TO(src.q[1],quaternion.q[1]);
				REAL_EQUAL_TO(src.q[2],quaternion.q[2]);
				REAL_EQUAL_TO(src.q[3],quaternion.q[3]);*/
			}
		}
	} END_TEST




	START_TEST("Geodetic coordinates (misc)") {
		EVDS_GEODETIC_COORDINATE geocoord;
		ERROR_CHECK(EVDS_Object_LoadFromString(root,
"<EVDS version=\"31\">"
"	<object name=\"Vessel\" type=\"vessel\" />"
"</EVDS>",&object));
		ERROR_CHECK(EVDS_Object_Initialize(object,1));

		EVDS_Geodetic_Set(&geocoord,object,0,-180.0,0);
		REAL_EQUAL_TO(geocoord.longitude,-180.0);

		EVDS_Geodetic_Set(&geocoord,object,0,0.0,0);
		REAL_EQUAL_TO(geocoord.longitude,0.0);

		EVDS_Geodetic_Set(&geocoord,object,0,180.0,0);
		REAL_EQUAL_TO(geocoord.longitude,-180.0);

		EVDS_Geodetic_Set(&geocoord,object,0,180.0 + 90.0,0);
		REAL_EQUAL_TO(geocoord.longitude,-90.0);

		EVDS_Geodetic_Set(&geocoord,object,0,360.0,0);
		REAL_EQUAL_TO(geocoord.longitude,0.0);

		EVDS_Geodetic_Set(&geocoord,object,0,360.0*1000.0 + 90.0,0);
		REAL_EQUAL_TO(geocoord.longitude,90.0);
		EVDS_Geodetic_Set(&geocoord,object,0,360.0*1000.0 - 45.0,0);
		REAL_EQUAL_TO(geocoord.longitude,-45.0);
		EVDS_Geodetic_Set(&geocoord,object,0,360.0*100000.0 + 25.0,0);
		REAL_EQUAL_TO(geocoord.longitude,25.0);
	} END_TEST


	START_TEST("Geodetic coordinates (around vessel)") {
		EVDS_GEODETIC_COORDINATE geocoord = { 0 };
		EVDS_GEODETIC_COORDINATE target;

		ERROR_CHECK(EVDS_Object_LoadFromString(root,
"<EVDS version=\"31\">"
"	<object name=\"Vessel\" type=\"vessel\" />"
"</EVDS>",&object));
		ERROR_CHECK(EVDS_Object_Initialize(object,1));

		//Distance from vessel of 1 km
		EVDS_Geodetic_Set(&geocoord,object,0,0,1000);

		//Check different coordinates
		geocoord.latitude = 90.0; //Elevation
		geocoord.longitude = 0.0;
		EVDS_Geodetic_ToVector(&vector,&geocoord);
		VECTOR_EQUAL_TO_EPS(&vector,0,0,1000.0,1e-7);
		EVDS_Geodetic_FromVector(&target,&vector,0);
		REAL_EQUAL_TO(target.latitude,geocoord.latitude);
		REAL_EQUAL_TO(target.longitude,geocoord.longitude);
		REAL_EQUAL_TO(target.elevation,geocoord.elevation);

		EQUAL_TO(vector.coordinate_system,object);

		geocoord.latitude = -90.0; //South pole
		geocoord.longitude = 0.0;
		EVDS_Geodetic_ToVector(&vector,&geocoord);
		VECTOR_EQUAL_TO_EPS(&vector,0,0,-1000.0,1e-7);
		EVDS_Geodetic_FromVector(&target,&vector,0);
		REAL_EQUAL_TO(target.latitude,geocoord.latitude);
		REAL_EQUAL_TO(target.longitude,geocoord.longitude);
		REAL_EQUAL_TO(target.elevation,geocoord.elevation);

		geocoord.latitude = 0.0; //Africa
		geocoord.longitude = 0.0;
		EVDS_Geodetic_ToVector(&vector,&geocoord);
		VECTOR_EQUAL_TO_EPS(&vector,1000.0,0,0,1e-7);
		EVDS_Geodetic_FromVector(&target,&vector,0);
		REAL_EQUAL_TO(target.latitude,geocoord.latitude);
		REAL_EQUAL_TO(target.longitude,geocoord.longitude);
		REAL_EQUAL_TO(target.elevation,geocoord.elevation);

		geocoord.latitude = 0.0; //East
		geocoord.longitude = 90.0;
		EVDS_Geodetic_ToVector(&vector,&geocoord);
		VECTOR_EQUAL_TO_EPS(&vector,0,1000.0,0,1e-7);
		EVDS_Geodetic_FromVector(&target,&vector,0);
		REAL_EQUAL_TO(target.latitude,geocoord.latitude);
		REAL_EQUAL_TO(target.longitude,geocoord.longitude);
		REAL_EQUAL_TO(target.elevation,geocoord.elevation);

		geocoord.latitude = 0.0; //Pacific ocean
		geocoord.longitude = -180.0;
		EVDS_Geodetic_ToVector(&vector,&geocoord);
		VECTOR_EQUAL_TO_EPS(&vector,-1000.0,0,0,1e-7);
		EVDS_Geodetic_FromVector(&target,&vector,0);
		REAL_EQUAL_TO(target.latitude,geocoord.latitude);
		REAL_EQUAL_TO(target.longitude,geocoord.longitude);
		REAL_EQUAL_TO(target.elevation,geocoord.elevation);

		geocoord.latitude = 0.0; //West
		geocoord.longitude = -90.0;
		EVDS_Geodetic_ToVector(&vector,&geocoord);
		VECTOR_EQUAL_TO_EPS(&vector,0,-1000.0,0,1e-7);
		EVDS_Geodetic_FromVector(&target,&vector,0);
		REAL_EQUAL_TO(target.latitude,geocoord.latitude);
		REAL_EQUAL_TO(target.longitude,geocoord.longitude);
		REAL_EQUAL_TO(target.elevation,geocoord.elevation);
	} END_TEST


	START_TEST("Geodetic coordinates (spheric planet)") {
		EVDS_GEODETIC_COORDINATE geocoord = { 0 };
		EVDS_GEODETIC_COORDINATE target;
		EVDS_OBJECT* earth;

		ERROR_CHECK(EVDS_Object_LoadFromString(root,
"<EVDS version=\"31\">"
"	<object name=\"Earth\" type=\"planet\">"
"		<parameter name=\"gravity.mu\">398600440000000</parameter>"
"		<parameter name=\"geometry.radius\">6378145.0</parameter>" //Spherical planet
"	</object>"
"</EVDS>",&earth));
		ERROR_CHECK(EVDS_Object_Initialize(earth,1));

		//All coordinates at 1 km elevation
		EVDS_Geodetic_Set(&geocoord,earth,0,0,1000);

		//Check different coordinates
		geocoord.latitude = 90.0; //North pole
		geocoord.longitude = 0.0;
		EVDS_Geodetic_ToVector(&vector,&geocoord);
		VECTOR_EQUAL_TO_EPS(&vector,0,0,6378145.0+1000.0,1e-7);
		EVDS_Geodetic_FromVector(&target,&vector,0);
		REAL_EQUAL_TO(target.latitude,geocoord.latitude);
		REAL_EQUAL_TO(target.longitude,geocoord.longitude);
		REAL_EQUAL_TO(target.elevation,geocoord.elevation);

		EQUAL_TO(vector.coordinate_system,earth);

		geocoord.latitude = -90.0; //South pole
		geocoord.longitude = 0.0;
		EVDS_Geodetic_ToVector(&vector,&geocoord);
		VECTOR_EQUAL_TO_EPS(&vector,0,0,-6378145.0-1000.0,1e-7);
		EVDS_Geodetic_FromVector(&target,&vector,0);
		REAL_EQUAL_TO(target.latitude,geocoord.latitude);
		REAL_EQUAL_TO(target.longitude,geocoord.longitude);
		REAL_EQUAL_TO(target.elevation,geocoord.elevation);

		geocoord.latitude = 0.0; //Africa
		geocoord.longitude = 0.0;
		EVDS_Geodetic_ToVector(&vector,&geocoord);
		VECTOR_EQUAL_TO_EPS(&vector,6378145.0+1000.0,0,0,1e-7);
		EVDS_Geodetic_FromVector(&target,&vector,0);
		REAL_EQUAL_TO(target.latitude,geocoord.latitude);
		REAL_EQUAL_TO(target.longitude,geocoord.longitude);
		REAL_EQUAL_TO(target.elevation,geocoord.elevation);

		geocoord.latitude = 0.0; //East
		geocoord.longitude = 90.0;
		EVDS_Geodetic_ToVector(&vector,&geocoord);
		VECTOR_EQUAL_TO_EPS(&vector,0,6378145.0+1000.0,0,1e-7);
		EVDS_Geodetic_FromVector(&target,&vector,0);
		REAL_EQUAL_TO(target.latitude,geocoord.latitude);
		REAL_EQUAL_TO(target.longitude,geocoord.longitude);
		REAL_EQUAL_TO(target.elevation,geocoord.elevation);

		geocoord.latitude = 0.0; //Pacific ocean
		geocoord.longitude = -180.0;
		EVDS_Geodetic_ToVector(&vector,&geocoord);
		VECTOR_EQUAL_TO_EPS(&vector,-6378145.0-1000.0,0,0,1e-7);
		EVDS_Geodetic_FromVector(&target,&vector,0);
		REAL_EQUAL_TO(target.latitude,geocoord.latitude);
		REAL_EQUAL_TO(target.longitude,geocoord.longitude);
		REAL_EQUAL_TO(target.elevation,geocoord.elevation);

		geocoord.latitude = 0.0; //West
		geocoord.longitude = -90.0;
		EVDS_Geodetic_ToVector(&vector,&geocoord);
		VECTOR_EQUAL_TO_EPS(&vector,0,-6378145.0-1000.0,0,1e-7);
		EVDS_Geodetic_FromVector(&target,&vector,0);
		REAL_EQUAL_TO(target.latitude,geocoord.latitude);
		REAL_EQUAL_TO(target.longitude,geocoord.longitude);
		REAL_EQUAL_TO(target.elevation,geocoord.elevation);
	} END_TEST


	START_TEST("Geodetic coordinates (oblate planet)") {
		EVDS_GEODETIC_COORDINATE geocoord = { 0 };
		EVDS_GEODETIC_COORDINATE target;
		EVDS_OBJECT* earth;

		ERROR_CHECK(EVDS_Object_LoadFromString(root,
"<EVDS version=\"31\">"
"	<object name=\"Earth\" type=\"planet\">"
"		<parameter name=\"gravity.mu\">398600440000000</parameter>"
"		<parameter name=\"geometry.semimajor_axis\">6378137.0</parameter>"
"		<parameter name=\"geometry.semiminor_axis\">6356752.0</parameter>"
"	</object>"
"</EVDS>",&earth));
		ERROR_CHECK(EVDS_Object_Initialize(earth,1));

		//All coordinates at 1 km elevation
		EVDS_Geodetic_Set(&geocoord,earth,0,0,1000);

		//Check different coordinates
		geocoord.latitude = 90.0; //North pole
		geocoord.longitude = 0.0;
		EVDS_Geodetic_ToVector(&vector,&geocoord);
		VECTOR_EQUAL_TO_EPS(&vector,0,0,6356752.0+1000.0,1e-7);
		EVDS_Geodetic_FromVector(&target,&vector,0);
		REAL_EQUAL_TO(target.latitude,geocoord.latitude);
		REAL_EQUAL_TO(target.longitude,geocoord.longitude);
		REAL_EQUAL_TO(target.elevation,geocoord.elevation);
		EQUAL_TO(vector.coordinate_system,earth);

		geocoord.latitude = -90.0; //South pole
		geocoord.longitude = 0.0;
		EVDS_Geodetic_ToVector(&vector,&geocoord);
		VECTOR_EQUAL_TO_EPS(&vector,0,0,-6356752.0-1000.0,1e-7);
		EVDS_Geodetic_FromVector(&target,&vector,0);
		REAL_EQUAL_TO(target.latitude,geocoord.latitude);
		REAL_EQUAL_TO(target.longitude,geocoord.longitude);
		REAL_EQUAL_TO(target.elevation,geocoord.elevation);

		geocoord.latitude = 0.0; //Africa
		geocoord.longitude = 0.0;
		EVDS_Geodetic_ToVector(&vector,&geocoord);
		VECTOR_EQUAL_TO_EPS(&vector,6378137.0+1000.0,0,0,1e-7);
		EVDS_Geodetic_FromVector(&target,&vector,0);
		REAL_EQUAL_TO(target.latitude,geocoord.latitude);
		REAL_EQUAL_TO(target.longitude,geocoord.longitude);
		REAL_EQUAL_TO(target.elevation,geocoord.elevation);

		geocoord.latitude = 0.0; //East
		geocoord.longitude = 90.0;
		EVDS_Geodetic_ToVector(&vector,&geocoord);
		VECTOR_EQUAL_TO_EPS(&vector,0,6378137.0+1000.0,0,1e-7);
		EVDS_Geodetic_FromVector(&target,&vector,0);
		REAL_EQUAL_TO(target.latitude,geocoord.latitude);
		REAL_EQUAL_TO(target.longitude,geocoord.longitude);
		REAL_EQUAL_TO(target.elevation,geocoord.elevation);

		geocoord.latitude = 0.0; //Pacific ocean
		geocoord.longitude = -180.0;
		EVDS_Geodetic_ToVector(&vector,&geocoord);
		VECTOR_EQUAL_TO_EPS(&vector,-6378137.0-1000.0,0,0,1e-7);
		EVDS_Geodetic_FromVector(&target,&vector,0);
		REAL_EQUAL_TO(target.latitude,geocoord.latitude);
		REAL_EQUAL_TO(target.longitude,geocoord.longitude);
		REAL_EQUAL_TO(target.elevation,geocoord.elevation);

		geocoord.latitude = 0.0; //West
		geocoord.longitude = -90.0;
		EVDS_Geodetic_ToVector(&vector,&geocoord);
		VECTOR_EQUAL_TO_EPS(&vector,0,-6378137.0-1000.0,0,1e-7);
		EVDS_Geodetic_FromVector(&target,&vector,0);
		REAL_EQUAL_TO(target.latitude,geocoord.latitude);
		REAL_EQUAL_TO(target.longitude,geocoord.longitude);
		REAL_EQUAL_TO(target.elevation,geocoord.elevation);
	} END_TEST



	START_TEST("Geodetic coordinates (oblate planet, numerical stability)") {
		EVDS_REAL lat,lon;
		EVDS_GEODETIC_COORDINATE geocoord = { 0 };
		EVDS_GEODETIC_COORDINATE target;
		EVDS_OBJECT* earth;

		ERROR_CHECK(EVDS_Object_LoadFromString(root,
"<EVDS version=\"31\">"
"	<object name=\"Earth\" type=\"planet\">"
"		<parameter name=\"gravity.mu\">398600440000000</parameter>"
"		<parameter name=\"geometry.semimajor_axis\">6378137.0</parameter>"
"		<parameter name=\"geometry.semiminor_axis\">6356752.0</parameter>"
"	</object>"
"</EVDS>",&earth));
		ERROR_CHECK(EVDS_Object_Initialize(earth,1));

		//All coordinates at 1 km elevation
		EVDS_Geodetic_Set(&geocoord,earth,0,0,1000);

		//Check numerical stability
		geocoord.latitude = EVDS_EPS;
		geocoord.longitude = 0.0;
		EVDS_Geodetic_ToVector(&vector,&geocoord);
		EVDS_Geodetic_FromVector(&target,&vector,0);
		REAL_EQUAL_TO_EPS(target.latitude,geocoord.latitude,EVDS_EPSf);
		REAL_EQUAL_TO_EPS(target.longitude,geocoord.longitude,EVDS_EPSf);
		REAL_EQUAL_TO_EPS(target.elevation,geocoord.elevation,EVDS_EPSf);

		geocoord.latitude = EVDS_EPS;
		geocoord.longitude = EVDS_EPS;
		EVDS_Geodetic_ToVector(&vector,&geocoord);
		EVDS_Geodetic_FromVector(&target,&vector,0);
		REAL_EQUAL_TO_EPS(target.latitude,geocoord.latitude,EVDS_EPSf);
		REAL_EQUAL_TO_EPS(target.longitude,geocoord.longitude,EVDS_EPSf);
		REAL_EQUAL_TO_EPS(target.elevation,geocoord.elevation,EVDS_EPSf);

		geocoord.latitude = 0.0;
		geocoord.longitude = EVDS_EPS;
		EVDS_Geodetic_ToVector(&vector,&geocoord);
		EVDS_Geodetic_FromVector(&target,&vector,0);
		REAL_EQUAL_TO_EPS(target.latitude,geocoord.latitude,EVDS_EPSf);
		REAL_EQUAL_TO_EPS(target.longitude,geocoord.longitude,EVDS_EPSf);
		REAL_EQUAL_TO_EPS(target.elevation,geocoord.elevation,EVDS_EPSf);


		geocoord.latitude = 90.0-100.0*EVDS_EPS;
		geocoord.longitude = 0.0;
		EVDS_Geodetic_ToVector(&vector,&geocoord);
		EVDS_Geodetic_FromVector(&target,&vector,0);
		REAL_EQUAL_TO_EPS(target.latitude,geocoord.latitude,EVDS_EPSf);
		REAL_EQUAL_TO_EPS(target.longitude,geocoord.longitude,EVDS_EPSf);
		REAL_EQUAL_TO_EPS(target.elevation,geocoord.elevation,EVDS_EPSf);

		geocoord.latitude = 90.0-100.0*EVDS_EPS;
		geocoord.longitude = 90.0;
		EVDS_Geodetic_ToVector(&vector,&geocoord);
		EVDS_Geodetic_FromVector(&target,&vector,0);
		REAL_EQUAL_TO_EPS(target.latitude,geocoord.latitude,EVDS_EPSf);
		REAL_EQUAL_TO_EPS(target.longitude,geocoord.longitude,EVDS_EPSf);
		REAL_EQUAL_TO_EPS(target.elevation,geocoord.elevation,EVDS_EPSf);

		geocoord.latitude = 90.0-100.0*EVDS_EPS;
		geocoord.longitude = -180.0;
		EVDS_Geodetic_ToVector(&vector,&geocoord);
		EVDS_Geodetic_FromVector(&target,&vector,0);
		REAL_EQUAL_TO_EPS(target.latitude,geocoord.latitude,EVDS_EPSf);
		REAL_EQUAL_TO_EPS(target.longitude,geocoord.longitude,EVDS_EPSf);
		REAL_EQUAL_TO_EPS(target.elevation,geocoord.elevation,EVDS_EPSf);

		geocoord.latitude = 90.0-100.0*EVDS_EPS;
		geocoord.longitude = -90.0;
		EVDS_Geodetic_ToVector(&vector,&geocoord);
		EVDS_Geodetic_FromVector(&target,&vector,0);
		REAL_EQUAL_TO_EPS(target.latitude,geocoord.latitude,EVDS_EPSf);
		REAL_EQUAL_TO_EPS(target.longitude,geocoord.longitude,EVDS_EPSf);
		REAL_EQUAL_TO_EPS(target.elevation,geocoord.elevation,EVDS_EPSf);


		geocoord.latitude = -90.0+100.0*EVDS_EPS;
		geocoord.longitude = 0.0;
		EVDS_Geodetic_ToVector(&vector,&geocoord);
		EVDS_Geodetic_FromVector(&target,&vector,0);
		REAL_EQUAL_TO_EPS(target.latitude,geocoord.latitude,EVDS_EPSf);
		REAL_EQUAL_TO_EPS(target.longitude,geocoord.longitude,EVDS_EPSf);
		REAL_EQUAL_TO_EPS(target.elevation,geocoord.elevation,EVDS_EPSf);

		geocoord.latitude = -90.0+100.0*EVDS_EPS;
		geocoord.longitude = 90.0;
		EVDS_Geodetic_ToVector(&vector,&geocoord);
		EVDS_Geodetic_FromVector(&target,&vector,0);
		REAL_EQUAL_TO_EPS(target.latitude,geocoord.latitude,EVDS_EPSf);
		REAL_EQUAL_TO_EPS(target.longitude,geocoord.longitude,EVDS_EPSf);
		REAL_EQUAL_TO_EPS(target.elevation,geocoord.elevation,EVDS_EPSf);

		geocoord.latitude = -90.0+100.0*EVDS_EPS;
		geocoord.longitude = -180.0;
		EVDS_Geodetic_ToVector(&vector,&geocoord);
		EVDS_Geodetic_FromVector(&target,&vector,0);
		REAL_EQUAL_TO_EPS(target.latitude,geocoord.latitude,EVDS_EPSf);
		REAL_EQUAL_TO_EPS(target.longitude,geocoord.longitude,EVDS_EPSf);
		REAL_EQUAL_TO_EPS(target.elevation,geocoord.elevation,EVDS_EPSf);

		geocoord.latitude = -90.0+100.0*EVDS_EPS;
		geocoord.longitude = -90.0;
		EVDS_Geodetic_ToVector(&vector,&geocoord);
		EVDS_Geodetic_FromVector(&target,&vector,0);
		REAL_EQUAL_TO_EPS(target.latitude,geocoord.latitude,EVDS_EPSf);
		REAL_EQUAL_TO_EPS(target.longitude,geocoord.longitude,EVDS_EPSf);
		REAL_EQUAL_TO_EPS(target.elevation,geocoord.elevation,EVDS_EPSf);


		geocoord.latitude = 90.0+100.0*EVDS_EPS;
		geocoord.longitude = 0.0;
		EVDS_Geodetic_ToVector(&vector,&geocoord);
		EVDS_Geodetic_FromVector(&target,&vector,0);
		REAL_EQUAL_TO_EPS(target.latitude,90.0,EVDS_EPSf);
		REAL_EQUAL_TO_EPS(target.longitude,-180.0,EVDS_EPSf);
		REAL_EQUAL_TO_EPS(target.elevation,geocoord.elevation,EVDS_EPSf);

		geocoord.latitude = -90.0-100.0*EVDS_EPS;
		geocoord.longitude = 0.0;
		EVDS_Geodetic_ToVector(&vector,&geocoord);
		EVDS_Geodetic_FromVector(&target,&vector,0);
		REAL_EQUAL_TO_EPS(target.latitude,-90.0,EVDS_EPSf);
		REAL_EQUAL_TO_EPS(target.longitude,-180.0,EVDS_EPSf);
		REAL_EQUAL_TO_EPS(target.elevation,geocoord.elevation,EVDS_EPSf);


		vector.x = 0.0;
		vector.y = 0.0;
		vector.z = 6356752.0 + 1000.0;
		EVDS_Geodetic_FromVector(&target,&vector,0);
		REAL_EQUAL_TO_EPS(target.latitude,90.0,EVDS_EPSf);
		REAL_EQUAL_TO_EPS(target.longitude,0.0,EVDS_EPSf);
		REAL_EQUAL_TO_EPS(target.elevation,geocoord.elevation,EVDS_EPSf);

		vector.x = 0.0;
		vector.y = 0.0;
		vector.z = - 6356752.0 - 1000.0;
		EVDS_Geodetic_FromVector(&target,&vector,0);
		REAL_EQUAL_TO_EPS(target.latitude,-90.0,EVDS_EPSf);
		REAL_EQUAL_TO_EPS(target.longitude,0.0,EVDS_EPSf);
		REAL_EQUAL_TO_EPS(target.elevation,geocoord.elevation,EVDS_EPSf);


		for (lat = -90.0; lat <= 90.0; lat += 10.0) {
			for (lon = -180.0; lon < 180.0; lon += 10.0) {
				geocoord.latitude = lat;
				geocoord.longitude = lon;
				EVDS_Geodetic_ToVector(&vector,&geocoord);
				EVDS_Geodetic_FromVector(&target,&vector,0);
				REAL_EQUAL_TO_EPS(target.latitude,geocoord.latitude,EVDS_EPSf);
				REAL_EQUAL_TO_EPS(target.longitude,geocoord.longitude,EVDS_EPSf);
				REAL_EQUAL_TO_EPS(target.elevation,geocoord.elevation,EVDS_EPSf);
			}
		}
	} END_TEST
}