#include "framework.h"

void Test_EVDS_FUNCTIONS() {
	START_TEST("Functions (general tests)") {
		EVDS_REAL x,y;
		ERROR_CHECK(EVDS_System_DatabaseFromString(system,
"<EVDS>"
"	<database name=\"combustion\">"
"		<entry name=\"H2-O2\" print=\"Oxygen with Hydrogen\">"
"			<parameter name=\"adiabatic_temperature\" interpolation=\"linear\">"
"				3.0 1000 K"
"				3.5 2000 K"
"				8.0 3000 K"
"				10.0 1000 K"
"				<data value=\"4.0\" interpolation=\"linear\">"
"					300.0 bar	2983.2 K"
"					325.0 bar	2984.7 K"
"					350.0 bar	2986.1 K"
"					375.0 bar	2987.3 K"
"					400.0 bar	2988.4 K"
"				</data>"
"				<data value=\"4.5\" interpolation=\"linear\">"
"					300.0 bar	3196.1 K"
"					325.0 bar	3198.8 K"
"					350.0 bar	3201.2 K"
"					375.0 bar	3203.4 K"
"					400.0 bar	3205.4 K"
"				</data>"
"				<data value=\"5.0\" interpolation=\"linear\">"
"					<data value=\"300.0 bar\">"
"						0.0	3375.4 K"
"						1.0	0.0"
"					</data>"
"					<data value=\"325.0 bar\">"
"						0.0	3379.6 K"
"						1.0	0.0"
"					</data>"
"					<data value=\"350.0 bar\">"
"						0.0	3383.4 K"
"						1.0	0.0"
"					</data>"
"					<data value=\"375.0 bar\">"
"						0.0	3386.8 K"
"						1.0	0.0"
"					</data>"
"					<data value=\"400.0 bar\">"
"						0.0	3390.0 K"
"						1.0	0.0"
"					</data>"
"				</data>"
"				<data value=\"5.5\" interpolation=\"linear\">"
"					300.0 bar	3521.9 K"
"					325.0 bar	3527.7 K"
"					350.0 bar	3533.0 K"
"					375.0 bar	3537.8 K"
"					400.0 bar	3542.2 K"
"				</data>"
"				<data value=\"6.0\" interpolation=\"linear\">"
"					300.0 bar	3635.8 K"
"					325.0 bar	3643.3 K"
"					350.0 bar	3650.1 K"
"					375.0 bar	3656.4 K"
"					400.0 bar	3662.2 K"
"				</data>"
"			</parameter>"
"		</entry>"
"	</database>"
"</EVDS>"));

		ERROR_CHECK(EVDS_System_GetDatabaseByName(system,"combustion",&variable));
		ERROR_CHECK(EVDS_Variable_GetNested(variable,"H2-O2",&variable));
		ERROR_CHECK(EVDS_Variable_GetNested(variable,"adiabatic_temperature",&variable));


		//Test 1D interpolation (in nodes)
		ERROR_CHECK(EVDS_Variable_GetFunctionValue(variable, 3.0, 300.0e5, 0.0, &y));
		REAL_EQUAL_TO(y, 1000.0);
		ERROR_CHECK(EVDS_Variable_GetFunctionValue(variable, 3.5, 300.0e5, 0.0, &y));
		REAL_EQUAL_TO(y, 2000.0);
		ERROR_CHECK(EVDS_Variable_GetFunctionValue(variable, 4.0, 300.0e5, 0.0, &y));
		REAL_EQUAL_TO(y, 2983.2);
		ERROR_CHECK(EVDS_Variable_GetFunctionValue(variable, 4.5, 300.0e5, 0.0, &y));
		REAL_EQUAL_TO(y, 3196.1);
		ERROR_CHECK(EVDS_Variable_GetFunctionValue(variable, 5.0, 300.0e5, 0.0, &y));
		REAL_EQUAL_TO(y, 3375.4);
		ERROR_CHECK(EVDS_Variable_GetFunctionValue(variable, 5.5, 300.0e5, 0.0, &y));
		REAL_EQUAL_TO(y, 3521.9);
		ERROR_CHECK(EVDS_Variable_GetFunctionValue(variable, 6.0, 300.0e5, 0.0, &y));
		REAL_EQUAL_TO(y, 3635.8);
		ERROR_CHECK(EVDS_Variable_GetFunctionValue(variable, 8.0, 300.0e5, 0.0, &y));
		REAL_EQUAL_TO(y, 3000);
		ERROR_CHECK(EVDS_Variable_GetFunctionValue(variable,10.0, 300.0e5, 0.0, &y));
		REAL_EQUAL_TO(y, 1000);


		//Test 2D interpolation (in nodes)
		ERROR_CHECK(EVDS_Variable_GetFunctionValue(variable, 4.5, 300.0e5, 0.0, &y));
		REAL_EQUAL_TO(y, 3196.1);
		ERROR_CHECK(EVDS_Variable_GetFunctionValue(variable, 4.5, 350.0e5, 0.0, &y));
		REAL_EQUAL_TO(y, 3201.2);
		ERROR_CHECK(EVDS_Variable_GetFunctionValue(variable, 4.5, 400.0e5, 0.0, &y));
		REAL_EQUAL_TO(y, 3205.4);

		ERROR_CHECK(EVDS_Variable_GetFunctionValue(variable, 5.0, 300.0e5, 0.0, &y));
		REAL_EQUAL_TO(y, 3375.4);
		ERROR_CHECK(EVDS_Variable_GetFunctionValue(variable, 5.0, 350.0e5, 0.0, &y));
		REAL_EQUAL_TO(y, 3383.4);
		ERROR_CHECK(EVDS_Variable_GetFunctionValue(variable, 5.0, 400.0e5, 0.0, &y));
		REAL_EQUAL_TO(y, 3390.0);

		ERROR_CHECK(EVDS_Variable_GetFunctionValue(variable, 5.5, 300.0e5, 0.0, &y));
		REAL_EQUAL_TO(y, 3521.9);
		ERROR_CHECK(EVDS_Variable_GetFunctionValue(variable, 5.5, 350.0e5, 0.0, &y));
		REAL_EQUAL_TO(y, 3533.0);
		ERROR_CHECK(EVDS_Variable_GetFunctionValue(variable, 5.5, 400.0e5, 0.0, &y));
		REAL_EQUAL_TO(y, 3542.2);


		//Test 3D interpolation (in nodes)
		ERROR_CHECK(EVDS_Variable_GetFunctionValue(variable, 5.0, 300.0e5, 0.0, &y));
		REAL_EQUAL_TO(y, 3375.4);
		ERROR_CHECK(EVDS_Variable_GetFunctionValue(variable, 5.0, 350.0e5, 0.0, &y));
		REAL_EQUAL_TO(y, 3383.4);
		ERROR_CHECK(EVDS_Variable_GetFunctionValue(variable, 5.0, 400.0e5, 0.0, &y));
		REAL_EQUAL_TO(y, 3390.0);

		ERROR_CHECK(EVDS_Variable_GetFunctionValue(variable, 5.0, 300.0e5, 1.0, &y));
		REAL_EQUAL_TO(y, 0.0);
		ERROR_CHECK(EVDS_Variable_GetFunctionValue(variable, 5.0, 350.0e5, 1.0, &y));
		REAL_EQUAL_TO(y, 0.0);
		ERROR_CHECK(EVDS_Variable_GetFunctionValue(variable, 5.0, 400.0e5, 1.0, &y));
		REAL_EQUAL_TO(y, 0.0);


		//Get an interpolated function
		//for (x = 2.8; x < 10.2; x += 0.1) {
		for (x = 4.5; x < 5.5; x += 0.05) {
			EVDS_REAL y1,y2,y3,y4;
			//EVDS_Variable_GetFunctionValue(variable,x,300.0e5,0.0,&y1);
			//EVDS_Variable_GetFunctionValue(variable,x,350.0e5,0.0,&y2);
			//EVDS_Variable_GetFunctionValue(variable,x,400.0e5,0.0,&y3);
			//EVDS_Variable_GetFunctionValue(variable,x,450.0e5,1.0,&y4);
			EVDS_Variable_GetFunctionValue(variable,x,300.0e5,0.0,&y1);
			EVDS_Variable_GetFunctionValue(variable,x,400.0e5,0.0,&y2);
			EVDS_Variable_GetFunctionValue(variable,x,300.0e5,0.5,&y3);
			EVDS_Variable_GetFunctionValue(variable,x,400.0e5,0.5,&y4);
			printf("%.2f = %.1f %.1f %.1f %.1f\n",x,y1,y2,y3,y4);
		}
		//EVDS_System_QueryDatabase

	} END_TEST




	START_TEST("Functions (extra multi-dimensional tests)") {
		EVDS_REAL y;
		EVDS_VARIABLE *A, *B, *C;
		ERROR_CHECK(EVDS_System_DatabaseFromString(system,
"<EVDS>"
"	<database name=\"combustion\">"
"		<entry name=\"H2-O2\" print=\"Oxygen with Hydrogen\">"
"			<parameter name=\"adiabatic_temperature1\" interpolation=\"linear\" order=\"xyz\" type=\"function\">"
"				4.0 2990 K"
"				4.5 3200 K"
"				6.0 3650 K"
"			</parameter>"
"			<parameter name=\"adiabatic_temperature2\" interpolation=\"linear\" order=\"yx\">"
"				<data value=\"300.0 bar\" interpolation=\"linear\">"
"					4.0		2983.2 K"
"					4.5		3196.1 K"
"					5.0		3375.4 K"
"					5.5		3521.9 K"
"					6.0		3635.8 K"
"				</data>"
"				<data value=\"350.0 bar\" interpolation=\"linear\">"
"					4.0		2986.1 K"
"					4.5		3201.2 K"
"					5.0		3383.4 K"
"					5.5		3533.0 K"
"					6.0		3650.1 K"
"				</data>"
"				<data value=\"400.0 bar\" interpolation=\"linear\">"
"					4.0		2988.4 K"
"					4.5		3205.4 K"
"					5.0		3390.0 K"
"					5.5		3542.2 K"
"					6.0		3662.2 K"
"				</data>"
"			</parameter>"
"			<parameter name=\"adiabatic_temperature3\" interpolation=\"linear\" order=\"zyx\">"
"				<data value=\"0.0\" constant=\"0.0\" />"
"				<data value=\"1.0\">"
"					<data value=\"300.0 bar\" interpolation=\"linear\">"
"						4.0		2983.2 K"
"						4.5		3196.1 K"
"						5.0		3375.4 K"
"						5.5		3521.9 K"
"						6.0		3635.8 K"
"					</data>"
"					<data value=\"350.0 bar\" interpolation=\"linear\">"
"						4.0		2986.1 K"
"						4.5		3201.2 K"
"						5.0		3383.4 K"
"						5.5		3533.0 K"
"						6.0		3650.1 K"
"					</data>"
"					<data value=\"400.0 bar\" interpolation=\"linear\">"
"						4.0		2988.4 K"
"						4.5		3205.4 K"
"						5.0		3390.0 K"
"						5.5		3542.2 K"
"						6.0		3662.2 K"
"					</data>"
"				</data>"
"			</parameter>"
"		</entry>"
"	</database>"
"</EVDS>"));

		ERROR_CHECK(EVDS_System_GetDatabaseByName(system,"combustion",&variable));
		ERROR_CHECK(EVDS_Variable_GetNested(variable,"H2-O2",&variable));
		ERROR_CHECK(EVDS_Variable_GetNested(variable,"adiabatic_temperature1",&A));
		ERROR_CHECK(EVDS_Variable_GetNested(variable,"adiabatic_temperature2",&B));
		ERROR_CHECK(EVDS_Variable_GetNested(variable,"adiabatic_temperature3",&C));

		//Test variable re-ordering
		ERROR_CHECK(EVDS_Variable_GetFunctionValue(A, 4.5, 350.0e5, 1.0, &y));
		REAL_EQUAL_TO(y, 3200);						  
		ERROR_CHECK(EVDS_Variable_GetFunctionValue(A, 4.5, 300.0e5, 1.0, &y));
		REAL_EQUAL_TO(y, 3200);						  
		ERROR_CHECK(EVDS_Variable_GetFunctionValue(A, 4.5, 400.0e5, 1.0, &y));
		REAL_EQUAL_TO(y, 3200);						  
		ERROR_CHECK(EVDS_Variable_GetFunctionValue(A, 4.5, 350.0e5, 0.5, &y));
		REAL_EQUAL_TO(y, 3200);						  
		ERROR_CHECK(EVDS_Variable_GetFunctionValue(A, 4.5, 300.0e5, 0.5, &y));
		REAL_EQUAL_TO(y, 3200);						  
		ERROR_CHECK(EVDS_Variable_GetFunctionValue(A, 4.5, 400.0e5, 0.5, &y));
		REAL_EQUAL_TO(y, 3200);

		ERROR_CHECK(EVDS_Variable_GetFunctionValue(B, 4.5, 350.0e5, 1.0, &y));
		REAL_EQUAL_TO(y, 3201.2);
		ERROR_CHECK(EVDS_Variable_GetFunctionValue(B, 4.5, 300.0e5, 1.0, &y));
		REAL_EQUAL_TO(y, 3196.1);
		ERROR_CHECK(EVDS_Variable_GetFunctionValue(B, 4.5, 400.0e5, 1.0, &y));
		REAL_EQUAL_TO(y, 3205.4);
		ERROR_CHECK(EVDS_Variable_GetFunctionValue(B, 4.5, 350.0e5, 0.5, &y));
		REAL_EQUAL_TO(y, 3201.2);
		ERROR_CHECK(EVDS_Variable_GetFunctionValue(B, 4.5, 300.0e5, 0.5, &y));
		REAL_EQUAL_TO(y, 3196.1);
		ERROR_CHECK(EVDS_Variable_GetFunctionValue(B, 4.5, 400.0e5, 0.5, &y));
		REAL_EQUAL_TO(y, 3205.4);

		ERROR_CHECK(EVDS_Variable_GetFunctionValue(C, 4.5, 350.0e5, 1.0, &y));
		REAL_EQUAL_TO(y, 3201.2);
		ERROR_CHECK(EVDS_Variable_GetFunctionValue(C, 4.5, 300.0e5, 1.0, &y));
		REAL_EQUAL_TO(y, 3196.1);
		ERROR_CHECK(EVDS_Variable_GetFunctionValue(C, 4.5, 400.0e5, 1.0, &y));
		REAL_EQUAL_TO(y, 3205.4);
		ERROR_CHECK(EVDS_Variable_GetFunctionValue(C, 4.5, 350.0e5, 0.5, &y));
		REAL_EQUAL_TO(y, 3201.2*0.5);
		ERROR_CHECK(EVDS_Variable_GetFunctionValue(C, 4.5, 300.0e5, 0.5, &y));
		REAL_EQUAL_TO(y, 3196.1*0.5);
		ERROR_CHECK(EVDS_Variable_GetFunctionValue(C, 4.5, 400.0e5, 0.5, &y));
		REAL_EQUAL_TO(y, 3205.4*0.5);


	} END_TEST
}