////////////////////////////////////////////////////////////////////////////////
/// @file
////////////////////////////////////////////////////////////////////////////////
/// Copyright (C) 2012-2015, Black Phoenix
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
/// @page EVDS_Solver_FuelTank Fuel tank
////////////////////////////////////////////////////////////////////////////////
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "evds.h"




////////////////////////////////////////////////////////////////////////////////
/// @brief Generate geometry for the fuel tank
////////////////////////////////////////////////////////////////////////////////
int EVDS_InternalFuelTank_GenerateGeometry(EVDS_OBJECT* object) {
	EVDS_VARIABLE* geometry;
	EVDS_REAL upper_radius = 0.0;
	EVDS_REAL lower_radius = 0.0;
	EVDS_REAL outer_radius = 0.0;
	EVDS_REAL inner_radius = 0.0;
	EVDS_REAL middle_length = 0.0;
	
	EVDS_Object_GetRealVariable(object,"geometry.upper_radius",&upper_radius,0);
	EVDS_Object_GetRealVariable(object,"geometry.lower_radius",&lower_radius,0);
	EVDS_Object_GetRealVariable(object,"geometry.outer_radius",&outer_radius,0);
	EVDS_Object_GetRealVariable(object,"geometry.inner_radius",&inner_radius,0);
	EVDS_Object_GetRealVariable(object,"geometry.middle_length",&middle_length,0);

	if ((upper_radius == 0.0) &&
		(lower_radius == 0.0) &&
		(outer_radius == 0.0) &&
		(inner_radius == 0.0) &&
		(middle_length == 0.0)) {
		return EVDS_OK;
	}

	//Reset the cross-sections for the engine
	if (EVDS_Object_GetVariable(object,"geometry.cross_sections",&geometry) == EVDS_OK) {
		EVDS_Variable_Destroy(geometry);
	}
	EVDS_Object_AddVariable(object,"geometry.cross_sections",EVDS_VARIABLE_TYPE_NESTED,&geometry);

	//Generate geometry for a normal fuel tank
	if (inner_radius <= 0.0) {
		EVDS_VARIABLE* upper_tip;
		EVDS_VARIABLE* upper_rim;
		EVDS_VARIABLE* lower_rim;
		EVDS_VARIABLE* lower_tip;

		//Add cross-sections
		EVDS_Variable_AddNested(geometry,"geometry.cross_sections",EVDS_VARIABLE_TYPE_NESTED,&upper_tip);
		EVDS_Variable_AddNested(geometry,"geometry.cross_sections",EVDS_VARIABLE_TYPE_NESTED,&upper_rim);
		EVDS_Variable_AddNested(geometry,"geometry.cross_sections",EVDS_VARIABLE_TYPE_NESTED,&lower_rim);
		EVDS_Variable_AddNested(geometry,"geometry.cross_sections",EVDS_VARIABLE_TYPE_NESTED,&lower_tip);

		//Radius
		EVDS_Variable_AddFloatAttribute(upper_tip,"r",inner_radius,0);
		EVDS_Variable_AddFloatAttribute(upper_rim,"r",outer_radius,0);
		EVDS_Variable_AddFloatAttribute(lower_rim,"r",outer_radius,0);
		EVDS_Variable_AddFloatAttribute(lower_tip,"r",inner_radius,0);

		//Tangents
		//EVDS_Variable_AddFloatAttribute(upper_tip,"tangent.radial.pos",0.0,0); //outer_radius
		EVDS_Variable_AddFloatAttribute(upper_rim,"tangent.offset.neg",upper_radius,0);
		EVDS_Variable_AddFloatAttribute(lower_rim,"tangent.offset.pos",lower_radius,0);
		//EVDS_Variable_AddFloatAttribute(lower_tip,"tangent.offset.pos",0.0,0);

		//Offsets
		EVDS_Variable_AddFloatAttribute(upper_tip,"offset",0.0,0);
		EVDS_Variable_AddFloatAttribute(upper_rim,"offset",upper_radius,0);
		EVDS_Variable_AddFloatAttribute(lower_rim,"offset",middle_length,0);
		EVDS_Variable_AddFloatAttribute(lower_tip,"offset",lower_radius,0);
	} else { //Geometry for a torus tank
		EVDS_VARIABLE* inner_rim_fwd;
		EVDS_VARIABLE* upper_tip;
		EVDS_VARIABLE* middle_rim_fwd;
		EVDS_VARIABLE* middle_rim_aft;
		EVDS_VARIABLE* lower_tip;
		EVDS_VARIABLE* inner_rim_aft;
		EVDS_VARIABLE* inner_middle;

		float half_radius = (float)((inner_radius + outer_radius) / 2);

		//Add cross-sections
		EVDS_Variable_AddNested(geometry,"geometry.cross_sections",EVDS_VARIABLE_TYPE_NESTED,&inner_rim_fwd );
		EVDS_Variable_AddNested(geometry,"geometry.cross_sections",EVDS_VARIABLE_TYPE_NESTED,&upper_tip     );
		EVDS_Variable_AddNested(geometry,"geometry.cross_sections",EVDS_VARIABLE_TYPE_NESTED,&middle_rim_fwd);
		EVDS_Variable_AddNested(geometry,"geometry.cross_sections",EVDS_VARIABLE_TYPE_NESTED,&middle_rim_aft);
		EVDS_Variable_AddNested(geometry,"geometry.cross_sections",EVDS_VARIABLE_TYPE_NESTED,&lower_tip     );
		EVDS_Variable_AddNested(geometry,"geometry.cross_sections",EVDS_VARIABLE_TYPE_NESTED,&inner_rim_aft );
		EVDS_Variable_AddNested(geometry,"geometry.cross_sections",EVDS_VARIABLE_TYPE_NESTED,&inner_middle  );

		//Radius
		EVDS_Variable_AddFloatAttribute(inner_rim_fwd ,"r",inner_radius,0);
		EVDS_Variable_AddFloatAttribute(upper_tip     ,"r",half_radius,0);
		EVDS_Variable_AddFloatAttribute(middle_rim_fwd,"r",outer_radius,0);
		EVDS_Variable_AddFloatAttribute(middle_rim_aft,"r",outer_radius,0);
		EVDS_Variable_AddFloatAttribute(lower_tip     ,"r",half_radius,0);
		EVDS_Variable_AddFloatAttribute(inner_rim_aft ,"r",inner_radius,0);
		EVDS_Variable_AddFloatAttribute(inner_middle  ,"r",inner_radius,0);

		//Tangents
		EVDS_Variable_AddFloatAttribute(inner_rim_fwd ,"tangent.offset.pos",-upper_radius,0); //outer_radius
		//EVDS_Variable_AddFloatAttribute(upper_tip     ,"tangent.offset.neg",upper_radius,0);
		EVDS_Variable_AddFloatAttribute(middle_rim_fwd,"tangent.offset.neg",upper_radius,0);
		EVDS_Variable_AddFloatAttribute(middle_rim_aft,"tangent.offset.pos",lower_radius,0);
		//EVDS_Variable_AddFloatAttribute(lower_tip     ,"tangent.offset.pos",lower_radius,0);
		EVDS_Variable_AddFloatAttribute(inner_rim_aft ,"tangent.offset.neg",-lower_radius,0);
		//EVDS_Variable_AddFloatAttribute(inner_middle  ,"tangent.offset.pos",lower_radius,0);

		//Offsets
		EVDS_Variable_AddFloatAttribute(inner_rim_fwd ,"offset", upper_radius,0);
		EVDS_Variable_AddFloatAttribute(upper_tip     ,"offset",-upper_radius,0);
		EVDS_Variable_AddFloatAttribute(middle_rim_fwd,"offset", upper_radius,0);
		EVDS_Variable_AddFloatAttribute(middle_rim_aft,"offset", middle_length,0);
		EVDS_Variable_AddFloatAttribute(lower_tip     ,"offset", lower_radius,0);
		EVDS_Variable_AddFloatAttribute(inner_rim_aft ,"offset",-lower_radius,0);
		EVDS_Variable_AddFloatAttribute(inner_middle  ,"offset",-middle_length,0);

		//Continuous
		EVDS_Variable_AddFloatAttribute(upper_tip     ,"continuous",1.0,0);
		//EVDS_Variable_AddFloatAttribute(middle_rim_fwd,"continuous",1.0,0);
		//EVDS_Variable_AddFloatAttribute(middle_rim_aft,"continuous",1.0,0);
		EVDS_Variable_AddFloatAttribute(lower_tip     ,"continuous",1.0,0);
		//EVDS_Variable_AddFloatAttribute(inner_rim_aft ,"continuous",1.0,0);
	}
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Update engine internal state
////////////////////////////////////////////////////////////////////////////////
int EVDS_InternalFuelTank_Solve(EVDS_SYSTEM* system, EVDS_SOLVER* solver, EVDS_OBJECT* object, EVDS_REAL delta_time) {
	EVDS_VARIABLE* v_mass;
	EVDS_VARIABLE* v_fuel_mass;
	EVDS_VARIABLE* v_total_mass;
	EVDS_REAL mass;
	EVDS_REAL fuel_mass;

	//Calculate total mass of the tank
	EVDS_ERRCHECK(EVDS_Object_GetVariable(object,"mass",&v_mass));
	EVDS_ERRCHECK(EVDS_Object_GetVariable(object,"fuel.mass",&v_fuel_mass));
	EVDS_ERRCHECK(EVDS_Object_GetVariable(object,"total_mass",&v_total_mass));

	//Set total mass
	EVDS_Variable_GetReal(v_mass,&mass);
	EVDS_Variable_GetReal(v_fuel_mass,&fuel_mass);
	EVDS_Variable_SetReal(v_total_mass,mass+fuel_mass);
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Initialize solver
////////////////////////////////////////////////////////////////////////////////
int EVDS_InternalFuelTank_Initialize(EVDS_SYSTEM* system, EVDS_SOLVER* solver, EVDS_OBJECT* object) {
	SIMC_LOCK_ID lock;
	EVDS_VARIABLE* variable;
	EVDS_REAL fuel_mass = 0.0;
	EVDS_REAL fuel_volume = 0.0;
	EVDS_REAL is_cryogenic = 0.0;
	EVDS_REAL load_ratio = 1.0;
	if (EVDS_Object_CheckType(object,"fuel_tank") != EVDS_OK) return EVDS_IGNORE_OBJECT; 

	//Generate geometry for the rocket engine
	EVDS_InternalFuelTank_GenerateGeometry(object);

	//Get total fuel load in percent
	if (EVDS_Object_GetVariable(object,"fuel.load_ratio",&variable) == EVDS_OK) {
		EVDS_Variable_GetReal(variable,&load_ratio);
	}
	if (EVDS_Object_GetVariable(object,"fuel.load_ratio_percent",&variable) == EVDS_OK) {
		EVDS_Variable_GetReal(variable,&load_ratio);
		load_ratio *= 0.01;
	}

	//Is fuel cryogenic
	if (EVDS_Object_GetVariable(object,"fuel.is_cryogenic",&variable) == EVDS_OK) {
		EVDS_Variable_GetReal(variable,&is_cryogenic);
	} else {
		is_cryogenic = 0;
		EVDS_Object_AddRealVariable(object,"fuel.is_cryogenic",0,0);
	}
	
	//Calculate total volume
	if (EVDS_Object_GetVariable(object,"fuel.volume",&variable) == EVDS_OK) {
		EVDS_Variable_GetReal(variable,&fuel_volume);
	} 
	if (fuel_volume < EVDS_EPS) {
		EVDS_MESH* mesh;
		EVDS_Mesh_Generate(object,&mesh,50.0f,EVDS_MESH_USE_DIVISIONS);
			fuel_volume = mesh->total_volume;
			EVDS_Object_AddRealVariable(object,"fuel.volume",0,&variable);
			EVDS_Variable_SetReal(variable,fuel_volume);
		EVDS_Mesh_Destroy(mesh);
	}

	//Calculate total mass
	if (EVDS_Object_GetVariable(object,"fuel.mass",&variable) == EVDS_OK) {
		EVDS_Variable_GetReal(variable,&fuel_mass);
	} else {
		//Specifying fuel capacity works just fine as specifying mass directly
		if (EVDS_Object_GetVariable(object,"fuel.capacity",&variable) == EVDS_OK) {
			EVDS_Variable_GetReal(variable,&fuel_mass);
		}
	}
	if (fuel_mass < EVDS_EPS) {
		EVDS_VARIABLE* material_database;
		EVDS_VARIABLE* material;
		EVDS_REAL fuel_density = 1000.0;

		//Check if material is defined
		if ((EVDS_Object_GetVariable(object,"fuel.type",&variable) == EVDS_OK) &&
			(EVDS_System_GetDatabaseByName(system,"material",&material_database) == EVDS_OK)) {
			char material_name[1024] = { 0 };
			EVDS_Variable_GetString(variable,material_name,1023,0);

			//Get material parameters
			if (EVDS_Variable_GetNested(material_database,material_name,&material) == EVDS_OK) {
				EVDS_REAL boiling_point = 0.0;
				EVDS_REAL fuel_temperature = 293.15; //20 C

				//Try liquid-phase fuel
				if (EVDS_Variable_GetNested(material,"boiling_point",&variable) == EVDS_OK) {
					EVDS_Variable_GetReal(variable,&boiling_point);
				}

				//Check if fuel is cryogenic or otherwise cooled down
				if (boiling_point < 273) {
					is_cryogenic = 1; //Force cryogenic fuel
					fuel_temperature = boiling_point - 0.1;

					EVDS_Object_GetVariable(object,"fuel.is_cryogenic",&variable);
					EVDS_Variable_SetReal(variable,1);
				}

				//Get density
				if (EVDS_Variable_GetNested(material,"density",&variable) == EVDS_OK) {
					EVDS_Variable_GetFunctionValue(variable,fuel_temperature,0,0,&fuel_density);
				}
			}

			//Calculate fuel mass
			fuel_mass = fuel_volume * fuel_density;
		}
		EVDS_Object_AddRealVariable(object,"fuel.mass",0,&variable);
		EVDS_Variable_SetReal(variable,fuel_mass);
	}

	//Remember the tank capacity
	EVDS_Object_AddRealVariable(object,"fuel.capacity",0,&variable);
	EVDS_Variable_SetReal(variable,fuel_mass);

	//Apply fuel load ratio
	if (load_ratio == 0.0) load_ratio = 1.0;
	if (load_ratio > 1.0) load_ratio = 1.0;
	if (load_ratio < 0.0) load_ratio = 0.0;
	EVDS_Object_GetVariable(object,"fuel.mass",&variable);
	EVDS_Variable_SetReal(variable,fuel_mass*load_ratio);

	//Fuel tanks have mass
	if (EVDS_Object_GetVariable(object,"mass",&variable) != EVDS_OK) {
		EVDS_Object_AddRealVariable(object,"mass",0,0);
	}
	EVDS_Object_AddRealVariable(object,"total_mass",0,0);

	//Add a mutex lock for EVDS_FuelTank_Consume()
	lock = SIMC_Lock_Create();
	EVDS_Object_SetSolverdata(object,lock);	
	return EVDS_CLAIM_OBJECT;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Deinitialize engine solver
////////////////////////////////////////////////////////////////////////////////
int EVDS_InternalFuelTank_Deinitialize(EVDS_SYSTEM* system, EVDS_SOLVER* solver, EVDS_OBJECT* object) {
	SIMC_LOCK_ID lock;
	EVDS_Object_GetSolverdata(object,&lock);
	SIMC_Lock_Destroy(lock);
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Consume fuel from a fuel tank.
///
/// This is a thread-safe call that consumes certain amount of fuel from the tank, and 
/// returns actual amount consumed (which can be less if fuel tank ran out of fuel).
///
/// @param[in] tank Pointer to fuel tank object
/// @param[in] amount Amount of propellant that must be consumed (in kg)
/// @param[out] consumed Actual amount of propellant that was consumed will be written here.
///
/// @returns Error code
/// @retval EVDS_OK Successfully completed
/// @retval EVDS_ERROR_BAD_PARAMETER "tank" is null
/// @retval EVDS_ERROR_BAD_PARAMETER "tank" is not a fuel tank
/// @retval EVDS_ERROR_BAD_PARAMETER "tank" does not have remaining fuel mass defined
////////////////////////////////////////////////////////////////////////////////
int EVDS_FuelTank_Consume(EVDS_OBJECT* tank, EVDS_REAL amount, EVDS_REAL* consumed) {
	SIMC_LOCK_ID lock;
	EVDS_VARIABLE* variable;
	EVDS_REAL fuel_mass;
	if (consumed) *consumed = 0.0; //No consumption by default
	if (!tank) return EVDS_ERROR_BAD_PARAMETER;
	if (EVDS_Object_CheckType(tank,"fuel_tank") != EVDS_OK) return EVDS_ERROR_BAD_PARAMETER;

	//Get the lock to prevent two threads from concurrently consuming fuel
	EVDS_Object_GetSolverdata(tank,&lock);
	SIMC_Lock_Enter(lock);

	//Get amount of fuel remaining
	if (EVDS_Object_GetVariable(tank,"fuel.mass",&variable) != EVDS_OK) {
		SIMC_Lock_Leave(lock);
		return EVDS_ERROR_BAD_PARAMETER;
	}
	EVDS_Variable_GetReal(variable,&fuel_mass);

	//Check if tank is already depleted
	if (fuel_mass <= 0.0) {
		SIMC_Lock_Leave(lock);
		return EVDS_OK;
	}

	//Consume some portion of fuel
	if (fuel_mass <= amount) { //Not enough fuel in this tank
		if (consumed) *consumed = fuel_mass;
		fuel_mass = 0.0;
	} else {
		if (consumed) *consumed = amount;
		fuel_mass -= amount;
	}
	if (fuel_mass < 0.0) fuel_mass = 0.0; //Clamp just in case
	EVDS_Variable_SetReal(variable,fuel_mass);

	//Unlock and return
	SIMC_Lock_Leave(lock);
	return EVDS_OK;
}




////////////////////////////////////////////////////////////////////////////////
EVDS_SOLVER EVDS_Solver_FuelTank = {
	EVDS_InternalFuelTank_Initialize, //OnInitialize
	EVDS_InternalFuelTank_Deinitialize, //OnDeinitialize
	EVDS_InternalFuelTank_Solve, //OnSolve
	0, //OnIntegrate
	0, //OnStateSave
	0, //OnStateLoad
	0, //OnStartup
	0, //OnShutdown
};
////////////////////////////////////////////////////////////////////////////////
/// @brief Register fuel tank solver
///
/// @param[in] system Pointer to EVDS_SYSTEM
///
/// @returns Error code
/// @retval EVDS_OK Successfully completed
/// @retval EVDS_ERROR_BAD_PARAMETER "system" is null
/// @retval EVDS_ERROR_BAD_STATE Cannot register solvers in current state
////////////////////////////////////////////////////////////////////////////////
int EVDS_FuelTank_Register(EVDS_SYSTEM* system) {
	return EVDS_Solver_Register(system,&EVDS_Solver_FuelTank);
}
