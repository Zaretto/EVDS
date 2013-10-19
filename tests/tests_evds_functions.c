#include "framework.h"

void Test_EVDS_FUNCTIONS() {
	START_TEST("Test") {
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
"					300.0 bar	3375.4 K"
"					325.0 bar	3379.6 K"
"					350.0 bar	3383.4 K"
"					375.0 bar	3386.8 K"
"					400.0 bar	3390.0 K"
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
			EVDS_Variable_GetFunction1D(variable,x,&y);
			printf("%.2f = %.0f\n",x,y);
		}
		//EVDS_System_QueryDatabase


	} END_TEST
}