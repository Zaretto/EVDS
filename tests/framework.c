////////////////////////////////////////////////////////////////////////////////
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
#include "framework.h"

void Test_Failure(char* expr, char* result, char* file, int line) {
	printf("FAIL %s == %s\nin %s at line %d\n",expr,result,file,line);
	EVDS_BREAKPOINT();
	exit(1);
}

void Test_Passed(char* expr, char* result, char* file, int line) {
	printf("PASS %s == %s\n",expr,result);
}

int Test_InList(void* ptr, SIMC_LIST* list) {
	SIMC_LIST_ENTRY* entry = SIMC_List_GetFirst(list);
	while (entry) {
		if (SIMC_List_GetData(list,entry) == ptr) {
			SIMC_List_Stop(list,entry);
			return 1;
		}
		entry = SIMC_List_GetNext(list,entry);
	}
	return 0;
}

void main() {
	//Test_EVDS_SYSTEM();
	//Test_EVDS_RIGID_BODY();
	//Test_EVDS_VECTOR();
	//Test_EVDS_QUATERNION();
	//Test_EVDS_FRAMES();
	//Test_EVDS_FUNCTIONS();
	//Test_EVDS_MODIFIER();
	//Test_EVDS_GIMBAL();
	//Test_EVDS_ROCKET_ENGINE();
	//Test_EVDS_WING();
	EVDS_SYSTEM* system;
	EVDS_OBJECT* root;
	EVDS_System_Create(&system);
	EVDS_RailroadSimulator_Register(system);
	EVDS_System_GetRootInertialSpace(system, &root);

	EVDS_OBJECT* object;
	EVDS_Object_LoadFromString(root,
		"<EVDS version=\"31\">"
		"    <object type=\"railroad.track\">"
		"        <parameter name=\"track.splines\">"
		"            <spline p0=\"-10 0 0\" t0=\"10 0 0\" p1=\"0 0 0\" t1=\"10 0 0\" />"
		"            <spline p1=\"0 10 0\" t1=\"0 10 0\" />"
		"            <spline p1=\"10 10 0\" t1=\"10 10 0\" />"
		"        </parameter>"
		"    </object>"
		"    <object type=\"railroad.track\">"
		"        <parameter name=\"track.splines\">"
		"            <spline p0=\"-100 -100 0\" t0=\"100 100 0\" p1=\"-10 0 0\" t1=\"10 0 0\" />"
		"        </parameter>"
		"    </object>"
		"</EVDS>", &object);
	EVDS_Object_Initialize(object, 1);

	getchar();
}