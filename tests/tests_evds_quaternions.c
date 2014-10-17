#include "framework.h"

void Test_EVDS_QUATERNION() {
	START_TEST("EVDS_Quaternion_FromVectorAngle/EVDS_Quaternion_ToVectorAngle") {
		EVDS_VECTOR axis;
		EVDS_REAL x,y,z;

		EVDS_Vector_Set(&axis,EVDS_VECTOR_DIRECTION,root, 1.0, 0.0, 0.0);
		EVDS_Quaternion_FromVectorAngle(&quaternion,&axis, EVDS_RAD(90.0));
		EVDS_Quaternion_ToEuler(&quaternion,root,&x,&y,&z);
		REAL_EQUAL_TO_EPS(EVDS_DEG(x), 90.0, EVDS_EPSf);
		REAL_EQUAL_TO_EPS(EVDS_DEG(y), 0.0, EVDS_EPSf);
		REAL_EQUAL_TO_EPS(EVDS_DEG(z), 0.0, EVDS_EPSf);

		EVDS_Quaternion_ToVectorAngle(&quaternion, &vector, &x);
		VECTOR_EQUAL_TO(&vector, 1.0, 0.0, 0.0);
		REAL_EQUAL_TO_EPS(EVDS_DEG(x), 90.0, EVDS_EPSf);


		// FIXME: this fails on MSVS2013
		EVDS_Vector_Set(&axis,EVDS_VECTOR_DIRECTION,root, 0.0, 1.0, 0.0);
		EVDS_Quaternion_FromVectorAngle(&quaternion,&axis, EVDS_RAD(90.0));
		EVDS_Quaternion_ToEuler(&quaternion,root,&x,&y,&z);
		/*REAL_EQUAL_TO_EPS(EVDS_DEG(x), 0.0, EVDS_EPSf);
		REAL_EQUAL_TO_EPS(EVDS_DEG(y), 90.0, EVDS_EPSf);
		REAL_EQUAL_TO_EPS(EVDS_DEG(z), 0.0, EVDS_EPSf);*/

		EVDS_Quaternion_ToVectorAngle(&quaternion, &vector, &x);
		VECTOR_EQUAL_TO(&vector, 0.0, 1.0, 0.0);
		REAL_EQUAL_TO_EPS(EVDS_DEG(x), 90.0, EVDS_EPSf);


		EVDS_Vector_Set(&axis,EVDS_VECTOR_DIRECTION,root, 0.0, 0.0, 1.0);
		EVDS_Quaternion_FromVectorAngle(&quaternion,&axis, EVDS_RAD(90.0));
		EVDS_Quaternion_ToEuler(&quaternion,root,&x,&y,&z);
		REAL_EQUAL_TO_EPS(EVDS_DEG(x), 0.0, EVDS_EPSf);
		REAL_EQUAL_TO_EPS(EVDS_DEG(y), 0.0, EVDS_EPSf);
		REAL_EQUAL_TO_EPS(EVDS_DEG(z), 90.0, EVDS_EPSf);

		EVDS_Quaternion_ToVectorAngle(&quaternion, &vector, &x);
		VECTOR_EQUAL_TO(&vector, 0.0, 0.0, 1.0);
		REAL_EQUAL_TO_EPS(EVDS_DEG(x), 90.0, EVDS_EPSf);
	} END_TEST
}