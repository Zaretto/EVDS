#include "framework.h"

void Test_EVDS_FUNCTIONS() {
	START_TEST("Test") {
		EVDS_REAL x;
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

		//Get an interpolated function
		for (x = 2.8; x < 10.2; x += 0.1) {
			EVDS_REAL y1,y2,y3,y4;
			EVDS_Variable_GetFunctionValue(variable,x,300.0e5,0.0,&y1);
			EVDS_Variable_GetFunctionValue(variable,x,350.0e5,0.0,&y2);
			EVDS_Variable_GetFunctionValue(variable,x,400.0e5,0.0,&y3);
			EVDS_Variable_GetFunctionValue(variable,x,450.0e5,1.0,&y4);
			printf("%.2f = %.1f %.1f %.1f %.1f\n",x,y1,y2,y3,y4);
		}
		//EVDS_System_QueryDatabase


	} END_TEST
}