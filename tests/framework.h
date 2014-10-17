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
#ifndef FRAMEWORK_H
#define FRAMEWORK_H

#define EVDS_LIBRARY

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <evds.h>

//#include <setjmp.h>
//#include <stdarg.h>
//jmp_buf *jumpBuf;

//Test function calls
void Test_Failure(char* expr, char* result, char* file, int line);
void Test_Passed(char* expr, char* result, char* file, int line);
int Test_InList(void* ptr, SIMC_LIST* list);

//Test-related macros
#define EQUAL_TO(expr,result) \
	if (expr == result) { \
		Test_Passed(#expr,#result,__FILE__,__LINE__); \
	} else { \
		Test_Failure(#expr,#result,__FILE__,__LINE__); \
	}

#define SILENT_EQUAL_TO(expr,result) \
	if (expr != result) { \
		Test_Failure(#expr,#result,__FILE__,__LINE__); \
	}

#define VECTOR_EQUAL_TO(vector,vx,vy,vz) \
	if ((fabs((vector)->x - (vx)) < EVDS_EPS) && \
		(fabs((vector)->y - (vy)) < EVDS_EPS) && \
		(fabs((vector)->z - (vz)) < EVDS_EPS)) { \
		Test_Passed(#vector,#vx" "#vy" "#vz,__FILE__,__LINE__); \
	} else { \
		Test_Failure(#vector,#vx" "#vy" "#vz,__FILE__,__LINE__); \
	}

#define VECTOR_EQUAL_TO_EPS(vector,vx,vy,vz,eps) \
	if ((fabs((vector)->x - (vx)) < eps) && \
		(fabs((vector)->y - (vy)) < eps) && \
		(fabs((vector)->z - (vz)) < eps)) { \
		Test_Passed(#vector,#vx" "#vy" "#vz,__FILE__,__LINE__); \
	} else { \
		Test_Failure(#vector,#vx" "#vy" "#vz,__FILE__,__LINE__); \
	}

#define REAL_EQUAL_TO(real,value) \
	if (fabs(real - value) < EVDS_EPS) { \
		Test_Passed(#real,#value,__FILE__,__LINE__); \
	} else { \
		Test_Failure(#real,#value,__FILE__,__LINE__); \
	}

#define REAL_EQUAL_TO_EPS(real,value,eps) \
	if (fabs(real - value) < eps) { \
		Test_Passed(#real,#value,__FILE__,__LINE__); \
	} else { \
		Test_Failure(#real,#value,__FILE__,__LINE__); \
	}

#define REAL_ANG_EQUAL_TO_EPS(real,value,eps) \
	if (fabs((real > 180.0 ? real - 360.0 : real)  - value) < eps) { \
		Test_Passed(#real, #value, __FILE__, __LINE__); \
	} else { \
		Test_Failure(#real, #value, __FILE__, __LINE__); \
	}

#define IS_IN_LIST(ptr,list) \
	if (Test_InList(ptr,list)) { \
		Test_Passed(#ptr,"IN "#list,__FILE__,__LINE__); \
	} else { \
		Test_Failure(#ptr,"IN "#list,__FILE__,__LINE__); \
	}

#define IS_NOT_IN_LIST(ptr,list) \
	if (!Test_InList(ptr,list)) { \
		Test_Passed(#ptr,"NOT IN "#list,__FILE__,__LINE__); \
	} else { \
		Test_Failure(#ptr,"NOT IN "#list,__FILE__,__LINE__); \
	}

#define STRING_EQUAL_TO(string,result) \
	if (strncmp(string,result,8192) == 0) { \
		Test_Passed(#string,#result,__FILE__,__LINE__); \
	} else { \
		Test_Failure(#string,#result,__FILE__,__LINE__); \
	}


#define ERROR_CHECK(expr) { \
	int error_code = expr; \
	SILENT_EQUAL_TO(error_code,EVDS_OK); }

#define START_TEST(name) { \
	EVDS_VECTOR vector = { 0 }; \
	EVDS_VECTOR vector1 = { 0 }; \
	EVDS_VECTOR vector2 = { 0 }; \
	EVDS_QUATERNION quaternion = { 0 }; \
	EVDS_QUATERNION quaternion1 = { 0 }; \
	EVDS_QUATERNION quaternion2 = { 0 }; \
	EVDS_STATE_VECTOR state; \
	EVDS_STATE_VECTOR_DERIVATIVE derivative; \
	EVDS_SYSTEM* system; \
	EVDS_OBJECT* object; \
	EVDS_VARIABLE* variable; \
	EVDS_REAL real; \
	SIMC_LIST* list; \
	SIMC_LIST_ENTRY* entry; \
	EVDS_OBJECT* root; \
	char string[8192] = { 0 }; \
	printf("\tTest: "name"\n"); \
	ERROR_CHECK(EVDS_System_Create(&system)); \
	ERROR_CHECK(EVDS_System_GetRootInertialSpace(system,&root)); \
	EVDS_Common_Register(system);

#define END_TEST EVDS_System_Destroy(system); }

#define NEED_ARBITRARY_OBJECT() \
	ERROR_CHECK(EVDS_Object_Create(root,&object));


//Various test files list
void Test_EVDS_SYSTEM();
void Test_EVDS_VECTOR();
void Test_EVDS_QUATERNION();
void Test_EVDS_FRAMES();
void Test_EVDS_FUNCTIONS();
void Test_EVDS_RIGID_BODY();
void Test_EVDS_MODIFIER();
void Test_EVDS_GIMBAL();
void Test_EVDS_ROCKET_ENGINE();
void Test_EVDS_WING();

//Disable annoying warnings
#pragma warning(disable: 4101)

#endif
