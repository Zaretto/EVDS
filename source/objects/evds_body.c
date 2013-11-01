////////////////////////////////////////////////////////////////////////////////
/// @file
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
/// @page EVDS_Solver_RigidBody Rigid body
///
/// Features
/// --------------------------------------------------------------------------------
/// The following features are inherited from EVDS_OBJECT initialization:
///  - Center of mass can be different from the reference point.
///  - Center of mass will be calculated from geometry unless specified explicitly.
///	 - Mass of the vessel/rigid body can be determined automatically from material
///	   that makes up the body.
///
/// Rigid body/vessel simulation features: 
///  - Center of mass affected by children objects (any object having mass will be accounted
///	   for).
///  - Supports variable mass effects (\f$\frac{dm}{dt} \neq 0\f$, variable center of mass).
///  - Change in mass computed from children objects (fuel tanks, etc) or computed as a derivative
///    of total mass automatically.
///  - Moments of inertia are considered quasiconstant (\f$\frac{dI}{dt} = 0\f$).
///  - Dynamic properties (moments of inertia, mass, center of mass) are assumed.
///		to have linear change over integration period (if a change occurs).
///
///	Additional advanced features:
///	 - Basic drag model for vessel and its children bodies which do not provide aerodynamic forces.
///	 - First-order realtime reentry heating model for vessel and its children.
///
///
/// Variables
/// --------------------------------------------------------------------------------
///	The following variables are automatically added by the rigid body solver. They represent totals
/// for the rigid body, including children bodies:
/// Name			| Description
/// ----------------|------------------------------------
///	total_cm		| Total center of mass
///	total_dcm		| Change in total center of mass (first derivative)
///	total_ix		| Total moment of inertia (X row)
///	total_iy		| Total moment of inertia (Y row)
///	total_iz		| Total moment of inertia (Z row)
///	total_inv_ix	| Inverse tensor of total moment of inertia (X row)
///	total_inv_iy	| Inverse tensor of total moment of inertia (Y row)
///	total_inv_iz	| Inverse tensor of total moment of inertia (Z row)
///	total_mass		| Total mass of the body
///	total_dmass		| Change in total mass of the body (first derivative)
///
/// Some of these variables must be specified for the rigid body simulation
///	(see EVDS_Object_Initialize() for more information):
/// Name			| Description
/// ----------------|------------------------------------
/// mass			| Vessel mass
///	jx				| Radius of gyration squared tensor (X row)
///	jy				| Radius of gyration squared tensor (Y row)
///	jz				| Radius of gyration squared tensor (Z row)
/// cm				| Center of mass
///	jxx				| Radius of gyration squared (principial axis X)
///	jyy				| Radius of gyration squared (principial axis Y)
///	jzz				| Radius of gyration squared (principial axis Z)
///	ixx				| Moment of inertia (principial axis X)
///	iyy				| Moment of inertia (principial axis Y)
///	izz				| Moment of inertia (principial axis Z)
///
/// 
/// Equations
/// --------------------------------------------------------------------------------
/// The rigid body simulation implements a set of equations listed below. These are 
/// all equations used for computing total parameters for the composite rigid body and
/// simulating forces and torques acting upon it.
///
/// ### Parallel Axis Theorem ###
/// This equation is used when total composite body moment of inertia is calculated. See
/// EVDS_Tensor_Rotate() for equations related to rotating moment of inertia tensor of the 
/// child body into rigid bodies coordinate system.
/// 
/// \f{eqnarray*}{
///		D &=& x^2 + y^2 + z^2\\
///		I_{total} &=& I_{total} + I_{child} + m 
///		\left[ \begin{array}{cccc} 
///			D - x^2 & 0 - x y & 0 - x z \\ 
///			0 - y x & D - y^2 & 0 - y z \\ 
///			0 - z x & 0 - z y & D - z^2
///		\end{array} \right]
/// \f}
///
/// where:
///  - \f$I_{total}\f$ is the total moment of inertia of the rigid body.
///  - \f$I_{child}\f$ is the total moment of inertia of the child body in rigid body coordinates
///  - \f$m\f$ is the mass of the child body.
///  - \f$x\f$, \f$y\f$, \f$z\f$ are coordinates of child body in rigid body coordinates.
///
/// ### Total Center of Mass ###
/// Total center of mass is calculated as a weighted average of all centers of mass.
///
/// \f{eqnarray*}{
///		CM &=& \frac{\sum m_i \cdot cm_i}{\sum m_i}
/// \f}
///
/// where:
///  - \f$CM\f$ is the total center of mass of the rigid body.
///  - \f$cm_i\f$ is the center of mass of the child body.
///  - \f$m_i\f$ is the mass of the child body.
///
/// ### Torque/Force from Applied Force ###
///
/// \f{eqnarray*}{
///		F_{total} &=& F_{total} + F \\
///		T_{total} &=& T_{total} + (F_{position} - CM) \times F
/// \f}
///
/// where:
///  - \f$F_{total}\f$ is the total force upon the rigid body center of mass.
///  - \f$T_{total}\f$ is the total torque upon the rigid body center of mass.
///  - \f$F\f$ is the applied force.
///  - \f$F_{position}\f$ is the force location in coordinates of the rigid body.
///  - \f$CM\f$ is the center of mass of the rigid body.
///
/// ### Torque/Force from Applied Torque ###
///
/// \f{eqnarray*}{
///		F_{total} &=& F_{total} + T \times (T_{position} - CM) \\
///		T_{total} &=& T_{total} + T
/// \f}
///
/// where:
///  - \f$F_{total}\f$ is the total force upon the rigid body center of mass.
///  - \f$T_{total}\f$ is the total torque upon the rigid body center of mass.
///  - \f$T\f$ is the applied torque.
///  - \f$T_{position}\f$ is the torque location in coordinates of the rigid body.
///  - \f$CM\f$ is the center of mass of the rigid body.
///
/// ### Acceleration from Total Force ###
///
/// \f{eqnarray*}{
///		a &=& F_{total} \cdot mass^{-1}
/// \f}
///
/// where:
///  - \f$a\f$ is the acceleration caused by forces upon the rigid body.
///  - \f$F_{total}\f$ is the total force upon the rigid body center of mass.
///  - \f$mass\f$ is the mass of the rigid body.
///
/// ### Angular Acceleration from Total Torque ###
/// Angular acceleration is computed in parent coordinate system assuming that torque
/// is specified in local coordinate system. The equation corresponds to Eulers equations
/// for a rigid body.
///
/// \f{eqnarray*}{
///		\alpha &=& I^{-1} [T_{total} - \omega \times (I \cdot \omega)]
/// \f}
///
/// where:
///  - \f$\alpha\f$ is the angular acceleration caused by torques upon rigid body.
///  - \f$T_{total}\f$ is the total torque upon the rigid body center of mass.
///  - \f$\omega\f$ is the angular velocity of the body.
///  - \f$I\f$ is the total moment of inertia.
///  - \f$I^{-1}\f$ is the inverse tensor of the total moment of inertia.
///
/// ### Gravity ###
/// See EVDS_Environment_GetGravitationalField() for equations related to acceleration due to
/// gravity equations. See EVDS_Callback_GetGravityGradientTorque() for equations related to
/// torque due to gravity gradient equations.
///
/// ### Aerodynamic Drag ###
/// See EVDS_Callback_GetAtmosphericData() for information about equations related to atmospheric
/// model.
///
///	### Solar Drag ###
/// See EVDS_Callback_GetRadiationData() for information about equations related to solar radiation
/// model.
///
/// ### Realtime Heating Model ###
/// (not implemented yet)
////////////////////////////////////////////////////////////////////////////////
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "evds.h"


#ifndef DOXYGEN_INTERNAL_STRUCTS
typedef struct EVDS_SOLVER_RIGID_USERDATA_TAG {
	//Is this body static?
	int is_static;
	
	//Is state consistent (has Solver been already called at least once)
	int is_consistent;

	//These variables are only for the body itself (initialized on first solver call)
	EVDS_VARIABLE *jx, *jy, *jz;	//Radius of gyration squared for this body (vectors building a tensor)
	EVDS_VARIABLE *cm;				//Center of mass for this vessel
	EVDS_VARIABLE *m;				//Mass for this vessel

	//These variables are for the body and all its children
	EVDS_VARIABLE *Ix, *Iy, *Iz;	//Total moment of inertia at current state (vectors building a tensor)
	EVDS_VARIABLE *Ix1, *Iy1, *Iz1;	//Inverse of total moment of inertia at current state (vectors building a tensor)
	EVDS_VARIABLE *M;				//Total mass
	EVDS_VARIABLE *dM;				//First derivative of total mass
	EVDS_VARIABLE *CM;				//Total center of mass
	EVDS_VARIABLE *dCM;				//First derivative of center of mass

	//Vessel-specific variables
	EVDS_VARIABLE *detach;			//Detach vessel from current parent
} EVDS_SOLVER_RIGID_USERDATA;
#endif




////////////////////////////////////////////////////////////////////////////////
/// @brief Rigid body solver
///
/// Calculates:
///  - Current moments of inertia, mass
///  - Position of center of mass according to all children
///  - Rate of change of mass, center of mass
///  - Forces acting from inside (engines, etc)
////////////////////////////////////////////////////////////////////////////////
int EVDS_InternalRigidBody_Solve(EVDS_SYSTEM* system, EVDS_SOLVER* solver, EVDS_OBJECT* object, EVDS_REAL delta_time) {
	//State variables
	EVDS_REAL M,dM;
	EVDS_REAL CMx,CMy,CMz;
	EVDS_REAL dCMx,dCMy,dCMz;
	EVDS_VECTOR Ix,Iy,Iz;
	EVDS_VECTOR Ix1,Iy1,Iz1;

	//Variables for child object
	EVDS_REAL m,dm,cmx,cmy,cmz,dcmx,dcmy,dcmz,x,y,z,D;
	EVDS_VECTOR cm,dcm;
	EVDS_VECTOR cIx,cIy,cIz;
	EVDS_STATE_VECTOR state;

	//List of children
	SIMC_LIST* children;
	SIMC_LIST_ENTRY* entry;
	EVDS_SOLVER_RIGID_USERDATA* userdata;
	EVDS_ERRCHECK(EVDS_Object_GetSolverdata(object,(void**)&userdata));

	//Fetch variables which have not yet been initialized
	if (!userdata->jx) EVDS_ERRCHECK(EVDS_Object_GetVariable(object,"jx",&userdata->jx));
	if (!userdata->jy) EVDS_ERRCHECK(EVDS_Object_GetVariable(object,"jy",&userdata->jy));
	if (!userdata->jz) EVDS_ERRCHECK(EVDS_Object_GetVariable(object,"jz",&userdata->jz));
	if (!userdata->cm) EVDS_ERRCHECK(EVDS_Object_GetVariable(object,"cm",&userdata->cm));
	userdata->is_consistent = 1;

	//Solve all children first
	EVDS_ERRCHECK(EVDS_Object_GetChildren(object,&children));
	entry = SIMC_List_GetFirst(children);
	while (entry) {
		EVDS_OBJECT* child = (EVDS_OBJECT*)SIMC_List_GetData(children,entry);
		EVDS_Object_Solve(child,delta_time);
		entry = SIMC_List_GetNext(children,entry);
	}

	//Prepare to accumulate all state variables
	EVDS_Variable_GetVector(userdata->cm,&cm);
	CMx = cm.x;		CMy = cm.y;		CMz = cm.z;
	dCMx = 0.0;		dCMy = 0.0;		dCMz = 0.0;

	//Compute tensor of inertia for this vessel
	EVDS_Variable_GetReal(userdata->m,&m);
	EVDS_Variable_GetVector(userdata->jx,&Ix);
	EVDS_Variable_GetVector(userdata->jy,&Iy);
	EVDS_Variable_GetVector(userdata->jz,&Iz);
	EVDS_Vector_Multiply(&Ix,&Ix,m); //I = j * mass
	EVDS_Vector_Multiply(&Iy,&Iy,m);
	EVDS_Vector_Multiply(&Iz,&Iz,m);
	M = m; dM = 0.0;

	//Accumulate variables in children
	entry = SIMC_List_GetFirst(children);
	while (entry) {
		int error_code;
		EVDS_VARIABLE* v_cm;
		EVDS_VARIABLE* v_mass;
		EVDS_VARIABLE *v_jx, *v_jy, *v_jz;
		EVDS_VARIABLE *v_ix, *v_iy, *v_iz;

		//Skip objects with no mass
		EVDS_OBJECT* child = (EVDS_OBJECT*)SIMC_List_GetData(children,entry);
		if ((EVDS_Object_GetVariable(child,"total_mass",&v_mass) != EVDS_OK) &&
			(EVDS_Object_GetVariable(child,"mass",&v_mass) != EVDS_OK)) {
			entry = SIMC_List_GetNext(children,entry);
			continue;
		}
		EVDS_Variable_GetReal(v_mass,&m);

		//Get center of mass
		if ((EVDS_Object_GetVariable(child,"total_cm",&v_cm) != EVDS_OK) &&
			(EVDS_Object_GetVariable(child,"cm",&v_cm) != EVDS_OK)) {
			entry = SIMC_List_GetNext(children,entry);
			continue;
		}
		EVDS_Variable_GetVector(v_cm,&cm);
		//EVDS_Variable_GetVector(child_userdata->dCM,&dcm);

		//Get moments of inertia
		error_code  = EVDS_Object_GetVariable(child,"total_ix",&v_ix);
		error_code += EVDS_Object_GetVariable(child,"total_iy",&v_iy);
		error_code += EVDS_Object_GetVariable(child,"total_iz",&v_iz);
		if (error_code != EVDS_OK) {
			if ((EVDS_Object_GetVariable(child,"jx",&v_jx) != EVDS_OK) ||
				(EVDS_Object_GetVariable(child,"jy",&v_jy) != EVDS_OK) ||
				(EVDS_Object_GetVariable(child,"jz",&v_jz) != EVDS_OK)) {
				entry = SIMC_List_GetNext(children,entry);
				continue;
			}

			EVDS_Variable_GetVector(v_jx,&Ix1);
			EVDS_Variable_GetVector(v_jy,&Iy1);
			EVDS_Variable_GetVector(v_jz,&Iz1);
			EVDS_Vector_Multiply(&Ix1,&Ix1,m);
			EVDS_Vector_Multiply(&Iy1,&Iy1,m);
			EVDS_Vector_Multiply(&Iz1,&Iz1,m);
		} else {
			EVDS_Variable_GetVector(v_ix,&Ix1);
			EVDS_Variable_GetVector(v_iy,&Iy1);
			EVDS_Variable_GetVector(v_iz,&Iz1);
		}

		//Convert CM to correct coordinates
		EVDS_Vector_Get(&cm,&cmx,&cmy,&cmz,object);
		//EVDS_Vector_Get(&dcm,&dcmx,&dcmy,&dcmz,object);

		//Get child position
		EVDS_Object_GetStateVector(child,&state);
		x = state.position.x;
		y = state.position.y;
		z = state.position.z;

		//Calculate new mass and center of mass
		CMx *= M; CMy *= M; CMz *= M;
		//dCMx *= M; dCMy *= M; dCMz *= M;

		M += m;
		//dM += dm;

		CMx = (CMx + m*cmx)/M;
		CMy = (CMy + m*cmy)/M;
		CMz = (CMz + m*cmz)/M;
		//dCMx = (dCMx + dm*dcmx)/M;
		//dCMy = (dCMy + dm*dcmy)/M;
		//dCMz = (dCMz + dm*dcmz)/M;

		//Rotate moments of inertia tensor into parent objects coordinates
		EVDS_Tensor_Rotate(&cIx,&cIy,&cIz,&Ix1,&Iy1,&Iz1,&state.orientation);

		//Apply parallel axis theorem
		D = x*x + y*y + z*z;
		cIx.x += m * (D - x*x);	cIx.y += m * (0 - x*y);	cIx.z += m * (0 - x*z);
		cIy.x += m * (0 - y*x);	cIy.y += m * (D - y*y);	cIy.z += m * (0 - y*z);
		cIz.x += m * (0 - z*x);	cIz.y += m * (0 - z*y);	cIz.z += m * (D - z*z);

		//Add to total
		EVDS_Vector_Add(&Ix,&Ix,&cIx);
		EVDS_Vector_Add(&Iy,&Iy,&cIy);
		EVDS_Vector_Add(&Iz,&Iz,&cIz);
		entry = SIMC_List_GetNext(children,entry);
	}

	//Store variables
	EVDS_Variable_SetVector(userdata->Ix,&Ix);
	EVDS_Variable_SetVector(userdata->Iy,&Iy);
	EVDS_Variable_SetVector(userdata->Iz,&Iz);
	EVDS_Variable_SetReal(userdata->M,M);
	EVDS_Variable_SetReal(userdata->dM,dM);

	EVDS_Vector_Set(&cm,EVDS_VECTOR_POSITION,object,CMx,CMy,CMz);
	EVDS_Vector_Set(&dcm,EVDS_VECTOR_POSITION,object,dCMx,dCMy,dCMz);
	EVDS_Variable_SetVector(userdata->CM,&cm);
	EVDS_Variable_SetVector(userdata->dCM,&dcm);

	//Build and store inverse of the inertia tensor
	EVDS_Tensor_InvertSymmetric(&Ix1,&Iy1,&Iz1,&Ix,&Iy,&Iz);
	EVDS_Variable_SetVector(userdata->Ix1,&Ix1);
	EVDS_Variable_SetVector(userdata->Iy1,&Iy1);
	EVDS_Variable_SetVector(userdata->Iz1,&Iz1);
	return EVDS_OK;
}




////////////////////////////////////////////////////////////////////////////////
/// @brief Rigid body integration routine. Outputs actual motion of the body
////////////////////////////////////////////////////////////////////////////////
int EVDS_InternalRigidBody_Integrate(EVDS_SYSTEM* system, EVDS_SOLVER* solver, EVDS_OBJECT* object,
									 EVDS_REAL delta_time, EVDS_STATE_VECTOR* state, EVDS_STATE_VECTOR_DERIVATIVE* derivative) {
	//State variables and parent coordinate system reference
	EVDS_OBJECT *parent_coordinates;
	EVDS_VECTOR cm,Ix,Iy,Iz,Ix1,Iy1,Iz1;
	EVDS_VECTOR Ga;
	EVDS_REAL mass;

	//Accumulation variables
	EVDS_VECTOR cm_force; //Total force at CM
	EVDS_VECTOR cm_torque; //Total torque at CM
	EVDS_VECTOR cm_a; //Total acceleration at CM
	EVDS_VECTOR cm_alpha; //Total angular acceleration at CM
	EVDS_VECTOR w; //Angular velocity in local coordinates
	EVDS_VECTOR Iw;

	//List of children
	SIMC_LIST* children;
	SIMC_LIST_ENTRY* entry;
	EVDS_SOLVER_RIGID_USERDATA* userdata;
	EVDS_ERRCHECK(EVDS_Object_GetSolverdata(object,(void**)&userdata));
	
	//Copy velocities, reset accelerations
	EVDS_Vector_Copy(&derivative->velocity,&state->velocity);
	EVDS_Vector_Copy(&derivative->angular_velocity,&state->angular_velocity);
	derivative->acceleration.x = 0;
	derivative->acceleration.y = 0;
	derivative->acceleration.z = 0;
	derivative->angular_acceleration.x = 0;
	derivative->angular_acceleration.y = 0;
	derivative->angular_acceleration.z = 0;

	//Prepare some variables
	EVDS_Variable_GetVector(userdata->CM,&cm);
	EVDS_Variable_GetReal(userdata->M,&mass);
	EVDS_Variable_GetVector(userdata->Ix,&Ix);
	EVDS_Variable_GetVector(userdata->Iy,&Iy);
	EVDS_Variable_GetVector(userdata->Iz,&Iz);
	EVDS_Variable_GetVector(userdata->Ix1,&Ix1);
	EVDS_Variable_GetVector(userdata->Iy1,&Iy1);
	EVDS_Variable_GetVector(userdata->Iz1,&Iz1);
	EVDS_Object_GetParent(object,&parent_coordinates); //Move in parent coordinates

	//Sanity check on mass
	if (mass <= EVDS_EPS) return EVDS_OK;

	//Calculate accelerations from forces & torques created by children objects
	EVDS_Vector_Initialize(cm_a);
	EVDS_Vector_Initialize(cm_alpha);
	EVDS_Vector_Initialize(w);
	EVDS_Vector_Initialize(Iw);

	//Begin accumulating forces
	EVDS_Vector_Set(&cm_force,EVDS_VECTOR_FORCE,object,0,0,0);
	EVDS_Vector_Set(&cm_torque,EVDS_VECTOR_TORQUE,object,0,0,0);

	//Iterate through children
	EVDS_ERRCHECK(EVDS_Object_GetChildren(object,&children));
	entry = SIMC_List_GetFirst(children);
	while (entry) {
		EVDS_VECTOR force;
		EVDS_VECTOR torque;
		EVDS_VECTOR force_position;
		EVDS_VECTOR torque_position;
		EVDS_STATE_VECTOR_DERIVATIVE child_derivative;
		EVDS_OBJECT* child = (EVDS_OBJECT*)SIMC_List_GetData(children,entry);

		//Get childrens forces (accelerations not supported)
		EVDS_Object_Integrate(child,delta_time,0,&child_derivative); 
		EVDS_Vector_Initialize(force);
		EVDS_Vector_Initialize(torque);
		EVDS_Vector_Initialize(force_position);
		EVDS_Vector_Initialize(torque_position);

		//------------------------------------------------------------------
		// Calculate force around current rigid bodies CM
		//------------------------------------------------------------------
		//Convert force into vessel coordinates
		EVDS_Vector_Convert(&force,&child_derivative.force,object);

		//Move force into center of mass, compute position relative to center of mass
		EVDS_Vector_GetPositionVector(&force,&force_position); //Get force position
		if (!force_position.coordinate_system) EVDS_Vector_Copy(&force_position,&cm);
		EVDS_Vector_SetPositionVector(&force,&cm); //Put force into center of mass
		EVDS_Vector_Subtract(&force_position,&force_position,&cm); //Find moment arm

		//Compute torque relative to center of mass
		EVDS_Vector_Cross(&torque,&force_position,&force);

		//Accumulate forces and torques
		EVDS_Vector_Add(&cm_force,&cm_force,&force);
		EVDS_Vector_Add(&cm_torque,&cm_torque,&torque);


		//------------------------------------------------------------------
		// Calculate torque around current rigid bodies CM
		//------------------------------------------------------------------
		//Convert torque into vessel coordinates
		/*EVDS_Vector_Convert(&torque,&child_derivative.torque,object);

		//Move torque into center of mass
		EVDS_Vector_GetPositionVector(&torque,&torque_position);
		if (!torque_position.coordinate_system) EVDS_Vector_Copy(&torque_position,&cm);
		EVDS_Vector_SetPositionVector(&torque,&cm);
		EVDS_Vector_Subtract(&torque_position,&torque_position,&cm);

		//Compute force relative to center of mass
		EVDS_Vector_Cross(&force,&torque,&torque_position);

		//Accumulate forces and torques
		//EVDS_Vector_Add(&cm_force,&cm_force,&force);
		//EVDS_Vector_Add(&cm_torque,&cm_torque,&torque);*/
		

		entry = SIMC_List_GetNext(children,entry);
	}


	//------------------------------------------------------------------
	// Convert force into acceleration
	//------------------------------------------------------------------
	//Store force (as force in center of mass) in the derivative
	EVDS_Vector_Copy(&derivative->force,&cm_force);
	EVDS_Vector_SetPositionVector(&derivative->force,&cm);

	//Convert force to linear acceleration (a = F/m)
	EVDS_Vector_Multiply(&cm_a,&cm_force,1/mass); //Apply scalar scale (1/m)
	cm_a.derivative_level = EVDS_VECTOR_ACCELERATION; //Force-change to acceleration vector
	EVDS_Vector_SetPositionVector(&cm_a,&cm); //Acceleration located in CM of rigid body

	//Convert acceleration from body coordinates to inertial coordinates
	EVDS_Vector_Convert(&cm_a,&cm_a,parent_coordinates);
	//At this point acceleration contains extra components due to motion of CM frame
	//an rotation of CM point in body frame. The will be removed later

	//Apply acceleration to the object
	EVDS_Vector_Add(&derivative->acceleration,&derivative->acceleration,&cm_a);

	//Subtract acceleration of CM-centered coordinate frame (FIXME: optimize this out)
	EVDS_Vector_Set(&cm_a,EVDS_VECTOR_ACCELERATION,object,0,0,0);
	EVDS_Vector_SetPositionVector(&cm_a,&cm);
	EVDS_Vector_Convert(&cm_a,&cm_a,parent_coordinates);
	EVDS_Vector_Subtract(&derivative->acceleration,&derivative->acceleration,&cm_a);


	//------------------------------------------------------------------
	// Convert torque into angular acceleration
	//------------------------------------------------------------------
	//Store torque (as torque in center of mass) in the derivative
	EVDS_Vector_Copy(&derivative->torque,&cm_torque);
	EVDS_Vector_SetPositionVector(&derivative->torque,&cm);

	//Compute angular acceleration in inertial coordinates
	//alpha_l = (I^-1) [T_l - w_l x (I*w_l)]
	EVDS_Vector_Convert(&w,&state->angular_velocity,object); //Calculate w (in local coordinates)
	EVDS_Tensor_MultiplyByVector(&Iw,&Ix,&Iy,&Iz,&w); //I*w
	EVDS_Vector_Cross(&Iw,&w,&Iw); //w x [I*w]
	Iw.derivative_level = EVDS_VECTOR_TORQUE; //Treat [w x (I*w)] as torque
	EVDS_Vector_Subtract(&Iw,&cm_torque,&Iw); //T - [w x (I*w)]
	EVDS_Tensor_MultiplyByVector(&cm_alpha,&Ix1,&Iy1,&Iz1,&Iw); //alpha = I^-1 [T - w x (I*w)]

	//Force alpha to be angular acceleration in CM of objects reference frame
	cm_alpha.derivative_level = EVDS_VECTOR_ANGULAR_ACCELERATION;
	EVDS_Vector_SetPositionVector(&cm_alpha,&cm);

	//Move angular acceleration to inertial coordinates
	EVDS_Vector_Convert(&cm_alpha,&cm_alpha,parent_coordinates);

	//Apply angular acceleration to the object
	EVDS_Vector_Add(&derivative->angular_acceleration,&derivative->angular_acceleration,&cm_alpha);

	//Subtract angular acceleration of CM-centered coordinate frame (FIXME: optimize this out)
	EVDS_Vector_Set(&cm_alpha,EVDS_VECTOR_ANGULAR_ACCELERATION,object,0,0,0);
	EVDS_Vector_SetPositionVector(&cm_alpha,&cm);
	EVDS_Vector_Convert(&cm_alpha,&cm_alpha,parent_coordinates);
	EVDS_Vector_Subtract(&derivative->angular_acceleration,&derivative->angular_acceleration,&cm_alpha);


	//------------------------------------------------------------------
	// Add fictious accelerations due to rotation around CM rather than body origin
	//------------------------------------------------------------------
	//Acceleration of bodies center of mass is zero in inertial coordinates (excluding additional forces)
	EVDS_Vector_Set(&cm_a,EVDS_VECTOR_ACCELERATION,parent_coordinates,0,0,0);
	EVDS_Vector_SetPositionVector(&cm_a,&cm);

	//Convert to local acceleration (acceleration corresponding to zero inertial in CM, in local frame)
	EVDS_Vector_Convert(&cm_a,&cm_a,object);
	EVDS_Vector_SetPositionVector(&cm_a,&state->position);

	//Add this acceleration to force center of mass acceleration to be zero
	EVDS_Vector_Convert(&cm_a,&cm_a,parent_coordinates);
	EVDS_Vector_Add(&derivative->acceleration,&derivative->acceleration,&cm_a);


	//------------------------------------------------------------------
	// Add additional forces (envrionmental forces)
	//------------------------------------------------------------------
	//Calculate acceleration due to gravity
	EVDS_Environment_GetGravitationalField(system,&state->position,0,&Ga);
	EVDS_Vector_Add(&derivative->acceleration,&derivative->acceleration,&Ga);


	//Do not move static bodies (FIXME: make static bodies more special)
	if (userdata->is_static) {
		derivative->acceleration.x = 0;
		derivative->acceleration.y = 0;
		derivative->acceleration.z = 0;
	}
	return EVDS_OK;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Initialize vessel solver
////////////////////////////////////////////////////////////////////////////////
int EVDS_InternalRigidBody_Initialize(EVDS_SYSTEM* system, EVDS_SOLVER* solver, EVDS_OBJECT* object) {
	int is_static = 0;
	EVDS_SOLVER_RIGID_USERDATA* userdata;
	EVDS_VECTOR temp;

	//Claim correct object type
	if (EVDS_Object_CheckType(object,"vessel") != EVDS_OK) {
		if (EVDS_Object_CheckType(object,"rigid_body") != EVDS_OK) {
			is_static = 1;
			if (EVDS_Object_CheckType(object,"static_body") != EVDS_OK) return EVDS_IGNORE_OBJECT; 
		}
	}

	//Create userdata
	userdata = (EVDS_SOLVER_RIGID_USERDATA*)malloc(sizeof(EVDS_SOLVER_RIGID_USERDATA));
	memset(userdata,0,sizeof(EVDS_SOLVER_RIGID_USERDATA));

	//Make sure the object has mass (FIXME: must make this object properly static)
	if (EVDS_Object_GetVariable(object,"mass",&userdata->m) != EVDS_OK) {
		//is_static = 1;
		EVDS_Object_AddRealVariable(object,"mass",0.0,&userdata->m);
	}

	//Set solverdata
	userdata->is_static = is_static;
	EVDS_ERRCHECK(EVDS_Object_SetSolverdata(object,userdata));

	//Inertia tensor components and center of mass will be fetched during first solver call
	userdata->jx = 0;
	userdata->jy = 0;
	userdata->jz = 0;
	userdata->cm = 0;
	userdata->is_consistent = 0;

	//Make sure runtime state variables exist
	EVDS_ERRCHECK(EVDS_Object_AddVariable(object,"total_cm",EVDS_VARIABLE_TYPE_VECTOR,&userdata->CM));
	EVDS_ERRCHECK(EVDS_Object_AddVariable(object,"total_dcm",EVDS_VARIABLE_TYPE_VECTOR,&userdata->dCM));
	EVDS_ERRCHECK(EVDS_Object_AddVariable(object,"total_ix",EVDS_VARIABLE_TYPE_VECTOR,&userdata->Ix));
	EVDS_ERRCHECK(EVDS_Object_AddVariable(object,"total_iy",EVDS_VARIABLE_TYPE_VECTOR,&userdata->Iy));
	EVDS_ERRCHECK(EVDS_Object_AddVariable(object,"total_iz",EVDS_VARIABLE_TYPE_VECTOR,&userdata->Iz));
	EVDS_ERRCHECK(EVDS_Object_AddVariable(object,"total_inv_ix",EVDS_VARIABLE_TYPE_VECTOR,&userdata->Ix1));
	EVDS_ERRCHECK(EVDS_Object_AddVariable(object,"total_inv_iy",EVDS_VARIABLE_TYPE_VECTOR,&userdata->Iy1));
	EVDS_ERRCHECK(EVDS_Object_AddVariable(object,"total_inv_iz",EVDS_VARIABLE_TYPE_VECTOR,&userdata->Iz1));
	EVDS_ERRCHECK(EVDS_Object_AddVariable(object,"total_mass",EVDS_VARIABLE_TYPE_FLOAT,&userdata->M));
	EVDS_ERRCHECK(EVDS_Object_AddVariable(object,"total_dmass",EVDS_VARIABLE_TYPE_FLOAT,&userdata->dM));

	//Vessel-specific variables
	if (EVDS_Object_CheckType(object,"vessel") == EVDS_OK) {
		EVDS_ERRCHECK(EVDS_Object_AddVariable(object,"detach",EVDS_VARIABLE_TYPE_FLOAT,&userdata->detach));
	}

	//Make sure all vectors are in correct coordinates
	EVDS_Vector_Set(&temp,EVDS_VECTOR_POSITION,object,0,0,0);
	EVDS_Variable_SetVector(userdata->CM,&temp);
	EVDS_Vector_Set(&temp,EVDS_VECTOR_VELOCITY,object,0,0,0);
	EVDS_Variable_SetVector(userdata->dCM,&temp);
	return EVDS_CLAIM_OBJECT;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Deinitialize vessel solver
////////////////////////////////////////////////////////////////////////////////
int EVDS_InternalRigidBody_Deinitialize(EVDS_SYSTEM* system, EVDS_SOLVER* solver, EVDS_OBJECT* object) {
	EVDS_SOLVER_RIGID_USERDATA* userdata;
	EVDS_ERRCHECK(EVDS_Object_GetSolverdata(object,(void**)&userdata));
	free(userdata);
	return EVDS_OK;
}




////////////////////////////////////////////////////////////////////////////////
/// @brief Update all vessels and detach them if required. Must be called by user to support "detach" variable for vessels.
///
/// @param[in] system EVDS system, for which vessel detaching must be processed
////////////////////////////////////////////////////////////////////////////////
int EVDS_RigidBody_UpdateDetaching(EVDS_SYSTEM* system) {	
	SIMC_LIST* list;
	SIMC_LIST_ENTRY* entry;

	//Check for every vessel object
	EVDS_ERRCHECK(EVDS_System_GetObjectsByType(system,"vessel",&list));
	entry = SIMC_List_GetFirst(list);
	while (entry) {
		EVDS_OBJECT* object = (EVDS_OBJECT*)SIMC_List_GetData(list,entry);
		EVDS_SOLVER_RIGID_USERDATA* userdata;
		EVDS_REAL detach;

		//Check if must be detached
		EVDS_ERRCHECK(EVDS_Object_GetSolverdata(object,(void**)&userdata));
		if (userdata->detach) {
			EVDS_Variable_GetReal(userdata->detach,&detach);
			if (detach > 0.5) { //Try to find propagator
				EVDS_OBJECT* parent;
				if (EVDS_Object_GetParentObjectByType(object,"propagator*",&parent) == EVDS_OK) {
					EVDS_Object_SetParent(object,parent);
				} else {
					if (EVDS_Object_GetParent(object,&parent) == EVDS_OK) {
						if (EVDS_Object_GetParent(parent,&parent) == EVDS_OK) {
							EVDS_Object_SetParent(object,parent); //Set to parent of the current parent
						}
					}
				}
			}
		}

		entry = SIMC_List_GetNext(list,entry);
	}
	return EVDS_OK;
}




////////////////////////////////////////////////////////////////////////////////
EVDS_SOLVER EVDS_Solver_RigidBody = {
	EVDS_InternalRigidBody_Initialize, //OnInitialize
	EVDS_InternalRigidBody_Deinitialize, //OnDeinitialize
	EVDS_InternalRigidBody_Solve, //OnSolve
	EVDS_InternalRigidBody_Integrate, //OnIntegrate
	0, //OnStateSave
	0, //OnStateLoad
	0, //OnStartup
	0, //OnShutdown
};
////////////////////////////////////////////////////////////////////////////////
/// @brief Register vessel solver
///
/// @param[in] system Pointer to EVDS_SYSTEM
///
/// @returns Error code
/// @retval EVDS_OK Successfully completed
/// @retval EVDS_ERROR_BAD_PARAMETER "system" is null
/// @retval EVDS_ERROR_BAD_STATE Cannot register solvers in current state
////////////////////////////////////////////////////////////////////////////////
int EVDS_RigidBody_Register(EVDS_SYSTEM* system) {
	return EVDS_Solver_Register(system,&EVDS_Solver_RigidBody);
}
