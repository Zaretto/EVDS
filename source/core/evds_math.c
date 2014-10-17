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
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "evds.h"


// Defines maximum distance between two coordinate systems for runtime conversion
#define EVDS_VECTOR_MAX_DEPTH 32


////////////////////////////////////////////////////////////////////////////////
/// @brief Convert vector to target coordinates (short conversion: two vectors are
///  located in coordinate systems, one of which is inside another)
////////////////////////////////////////////////////////////////////////////////
void EVDS_Vector_ShortConvert(EVDS_VECTOR* target, EVDS_VECTOR* vector, EVDS_OBJECT* target_coordinates) {
	int target_is_child = 0;
	//Child and parent coordinate systems
	EVDS_OBJECT* child_coordinates;
	EVDS_OBJECT* parent_coordinates;
	//Child state vector
	EVDS_STATE_VECTOR* child_state;
	//Vector coordinate system
	EVDS_OBJECT* vector_coordinates;
	vector_coordinates = vector->coordinate_system;

	//Verify inputs are correct
	EVDS_ASSERT(
		(target_coordinates->parent == vector_coordinates) ||	//Target coordinates are child of vector coordinates
		(target_coordinates == vector_coordinates->parent)		//Target coordinates are a parent of vector coordinates
	);

	//Determine child and parent coordinate systems
	if (vector_coordinates == target_coordinates->parent) {
		//Target coordinates are child of vector coordinates
		//This means "vector" has coordinate system (parent) in which all transformations are made 
		//(before rotating vector into target coordinates)
		parent_coordinates = vector_coordinates; //a
		child_coordinates = target_coordinates; //b
		target_is_child = 1;
	} else { //else if (target_coordinates == vector_coordinates->parent) {
		//Target coordinates are a parent of vector coordinates
		//This means all transformations must be done in parent
		parent_coordinates = target_coordinates; //a
		child_coordinates = vector_coordinates; //b
		target_is_child = 0;
	}

	//Get correct state vectors (differentiate between public and private state vector)
#ifndef EVDS_SINGLETHREADED
	if (SIMC_Thread_GetUniqueID() == child_coordinates->integrate_thread) {
		child_state = &child_coordinates->private_state;
	} else if (SIMC_Thread_GetUniqueID() == child_coordinates->render_thread) {
		child_state = &child_coordinates->render_state;
	} else {
		child_state = &child_coordinates->state;
	}
#endif

	//Assert consistency
	EVDS_ASSERT(parent_coordinates == child_coordinates->parent);
	EVDS_ASSERT(child_state->orientation.coordinate_system == parent_coordinates);

	//Rotate from child to parent
	if (!target_is_child) {
		EVDS_Vector_Rotate(target,vector,&child_state->orientation);
	} else {
		EVDS_Vector_Copy(target,vector);
	}

	//Convert vector position into target coordinates
	if (vector->pcoordinate_system) {
		EVDS_VECTOR position;
		EVDS_Vector_GetPositionVector(vector,&position);
		EVDS_Vector_Convert(&position,&position,target_coordinates);
		EVDS_Vector_SetPositionVector(target,&position);
	}

	//Convert vector velocity into target coordinates
	if (vector->vcoordinate_system) {
		EVDS_VECTOR velocity;
		EVDS_Vector_GetVelocityVector(vector,&velocity);
		EVDS_Vector_Convert(&velocity,&velocity,target_coordinates);
		EVDS_Vector_SetVelocityVector(target,&velocity);
	}

	//Execute transformation
	switch (vector->derivative_level) {
		case EVDS_VECTOR_INERTIAL_TRANSFORM:
		case EVDS_VECTOR_ANGULAR_VELOCITY:
		case EVDS_VECTOR_ANGULAR_ACCELERATION: 
		case EVDS_VECTOR_FORCE:
		case EVDS_VECTOR_TORQUE: {
			//Do nothing
		} break;
		case EVDS_VECTOR_POSITION: {
			EVDS_ASSERT(child_state->position.coordinate_system == parent_coordinates);
			if (!target_is_child) {
				//r[P/a] = r[P/b] + r[Q/a]
				EVDS_Vector_Add(target,target,&child_state->position);
			} else { 
				//r[P/b] = r[P/a] - r[Q/a]
				EVDS_Vector_Subtract(target,target,&child_state->position);
			}
		} break;
		case EVDS_VECTOR_VELOCITY: {
			EVDS_VECTOR vector_position;
			EVDS_VECTOR coriolis_component;
			EVDS_VECTOR extra_components;

			EVDS_Vector_Initialize(vector_position);
			EVDS_Vector_Initialize(coriolis_component);
			EVDS_Vector_Initialize(extra_components);

			EVDS_ASSERT(child_state->position.coordinate_system == parent_coordinates);
			EVDS_ASSERT(child_state->angular_velocity.coordinate_system == parent_coordinates);
			EVDS_ASSERT(child_state->velocity.coordinate_system == parent_coordinates);

			if (vector->pcoordinate_system) {
				EVDS_Vector_GetPositionVector(vector,&vector_position);
				EVDS_Vector_Convert(&vector_position,&vector_position,parent_coordinates); //FIXME: doing same twice
				EVDS_Vector_Subtract(&vector_position,&vector_position,&child_state->position);
			} else {
				vector_position.coordinate_system = parent_coordinates;
				vector_position.derivative_level = EVDS_VECTOR_POSITION;
			}
			EVDS_Vector_Cross(&coriolis_component,&child_state->angular_velocity,&vector_position);
			EVDS_Vector_Add(&extra_components,&child_state->velocity,&coriolis_component);
			if (!target_is_child) {
				//v[P/a] = v[P/b] + V[Q/a] + w[b/a] x r[P/b]
				//v[P/a] = V[P/b] + (V[Q/a] + w[b/a] x (r[P/a] - r[Q/a]))
				EVDS_Vector_Add(target,target,&extra_components);
			} else {
				//v[P/b] = V[P/a] - V[Q/a] - w[b/a] x r[P/b]
				//v[P/b] = V[P/a] - (V[Q/a] + w[b/a] x (r[P/a] - r[Q/a]))
				EVDS_Vector_Subtract(target,target,&extra_components);
			}
		} break;
		case EVDS_VECTOR_ACCELERATION: {
			EVDS_VECTOR vector_position;
			EVDS_VECTOR vector_velocity;
			EVDS_VECTOR coriolis_component;
			EVDS_VECTOR centripetal_component;
			EVDS_VECTOR alpha_component;
			EVDS_VECTOR extra_components;

			EVDS_Vector_Initialize(vector_position);
			EVDS_Vector_Initialize(vector_velocity);
			EVDS_Vector_Initialize(coriolis_component);
			EVDS_Vector_Initialize(centripetal_component);
			EVDS_Vector_Initialize(alpha_component);
			EVDS_Vector_Initialize(extra_components);

			EVDS_ASSERT(child_state->position.coordinate_system == parent_coordinates);
			EVDS_ASSERT(child_state->angular_velocity.coordinate_system == parent_coordinates);
			EVDS_ASSERT(child_state->angular_acceleration.coordinate_system == parent_coordinates);
			EVDS_ASSERT(child_state->velocity.coordinate_system == parent_coordinates);
			EVDS_ASSERT(child_state->acceleration.coordinate_system == parent_coordinates);

			if (vector->pcoordinate_system) {
				EVDS_Vector_GetPositionVector(vector,&vector_position);
				EVDS_Vector_Convert(&vector_position,&vector_position,parent_coordinates); //FIXME: doing same twice
				EVDS_Vector_Subtract(&vector_position,&vector_position,&child_state->position);
			} else {
				vector_position.coordinate_system = parent_coordinates;
				vector_position.derivative_level = EVDS_VECTOR_POSITION;
			}

			//Get velocity of point in parent coordinates
			if (vector->vcoordinate_system) {
				EVDS_Vector_GetVelocityVector(vector,&vector_velocity);
				EVDS_Vector_Convert(&vector_velocity,&vector_velocity,parent_coordinates); //FIXME: doing same twice
			} else {
				vector_velocity.coordinate_system = parent_coordinates;
				vector_velocity.derivative_level = EVDS_VECTOR_VELOCITY;
			}

			//w'[b/a] x (r[P/a] - r[Q/a])
			EVDS_Vector_Cross(&alpha_component,&child_state->angular_acceleration,&vector_position);
			//w[b/a] x (w[b/a] x (r[P/a] - r[Q/a]))
			EVDS_Vector_Cross(&centripetal_component,&child_state->angular_velocity,&vector_position);
			EVDS_Vector_Cross(&centripetal_component,&child_state->angular_velocity,&centripetal_component);
			//2 w[b/a] x V[P/b]
			EVDS_Vector_Cross(&coriolis_component,&child_state->angular_velocity,&vector_velocity);
			EVDS_Vector_Multiply(&coriolis_component,&coriolis_component,2.0);

			EVDS_Vector_Add(&extra_components,&child_state->acceleration,&alpha_component);
			EVDS_Vector_Add(&extra_components,&extra_components,&centripetal_component);
			EVDS_Vector_Add(&extra_components,&extra_components,&coriolis_component);
			if (!target_is_child) {
				//a[P/a] = a[P/b] + a[Q/a] + w'[b/a] x r[P/b] + w[b/a] x (w[b/a] x r[P/b]) + 2 w[b/a] x V[P/b]
				//a[P/a] = a[P/b] + a[Q/a] + w'[b/a] x (r[P/a] - r[Q/a]) + w[b/a] x (w[b/a] x (r[P/a] - r[Q/a])) + 2 w[b/a] x V[P/b]
				EVDS_Vector_Add(target,target,&extra_components);
			} else {
				//a[P/b] = a[P/a] - (a[Q/a] + w'[b/a] x r[P/b] + w[b/a] x (w[b/a] x r[P/b]) + 2 w[b/a] x V[P/b])
				//a[P/b] = a[P/a] - (a[Q/a] + w'[b/a] x (r[P/a] - r[Q/a]) + w[b/a] x (w[b/a] x (r[P/a] - r[Q/a])) + 2 w[b/a] x V[P/b])
				EVDS_Vector_Subtract(target,target,&extra_components);
			}
		} break;
	}

	if (target_is_child) { //Not used when coordinate systems match
		//Rotate from parent to child
		EVDS_Vector_RotateConjugated(target,target,&child_state->orientation);
	}

	//Set coordinate system in target vector
	target->derivative_level = vector->derivative_level;
	target->coordinate_system = target_coordinates;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Convert vector to target coordinates.
///
/// This routine converts between coordinate system with regard to their relative
/// velocities, accelerations, and other non-inertial effects. The conversion is supported
/// between any available vector types.
///
/// The conversion is performed using short conversion between two coordinate systems that are
/// a parent and a child (therefore there is no traversal penalty for converting between such
/// coordinate systems).
/// Child coordinate system is the one that's a child in object hierarchy, and has its state specified in coordinates
/// of the parent system.
///
/// In other case, a traversal is made to perform conversion through a shared parent. In case 
/// there is no shared parent, the result is undefined.
///
/// The equations used for conversion are listed below. In these equations, subscript like \f$x_{P/b}\f$
/// means vector in point \f$P\f$ in coordinate system \f$b\f$. Coordinate system \f$a\f$ is the parent coordinate system,
/// and coordinate system \f$b\f$ is the child (nested) coordinate system.
///
/// In these equations, \f$r\f$ is a position vector, \f$v\f$ is a velocity vector, \f$a\f$ is the acceleration vector,
/// \f$\omega\f$ is angular velocity, \f$\alpha\f$ is angular acceleration. Point \f$P\f$ is the location of converted vector,
/// point \f$Q\f$ is the origin of child coordinate system.
///
/// Equations list corresponding equation number from "Aircraft Control and Simulation" (B.L.Stevens, F.L.Lewis, 2003).
///
/// \f{eqnarray*}{
///		r_{P/a} &=& r_{P/b} + r_{Q/a} \tag{1.2-8} \\
///		v_{P/a} &=& v_{P/b} + v_{Q/a} + \omega_{b/a} \times r_{P/b} \tag{1.2-10} \\
///		a_{P/a} &=& a_{P/b} + a_{Q/a} + \alpha_{b/a} \times r_{P/b} + 
///			\omega_{b/a} \times (\omega_{b/a} \times r_{P/b}) + 2 \omega_{b/a} \times v_{P/b} \tag{1.2-11}
/// \f}
///
/// Position and velocity of the given vector are also transformed into target coordinate system.
///
/// @param[out] target Vector, where result will be written
/// @param[in] v Vector, that must be converted
/// @param[in] target_coordinates Target coordinates, to which the vector must be converted
/// @returns Vector in target coordinates
////////////////////////////////////////////////////////////////////////////////
void EVDS_Vector_Convert(EVDS_VECTOR* target, EVDS_VECTOR* v, EVDS_OBJECT* target_coordinates) {
	//Do not convert if already in correct coordinates
	if (target_coordinates == v->coordinate_system) {
		if (target != v) memcpy(target,v,sizeof(EVDS_VECTOR));
		return;
	}

	//Make sure coordinate system is defined for the vector to be converted
	EVDS_ASSERT(v->coordinate_system != 0);
	EVDS_ASSERT(target_coordinates != 0);

	//Execute short conversion
	if ((target_coordinates->parent == v->coordinate_system) ||
		(target_coordinates == v->coordinate_system->parent)) {
		EVDS_Vector_ShortConvert(target,v,target_coordinates);
	} else {
		//Used for back-tracking when target is deeper than vector
		EVDS_OBJECT* parent_track[EVDS_VECTOR_MAX_DEPTH];
		int parent_depth = 0;

		//Temporary vector and parent levels (at what level of tree vectors are)
		EVDS_VECTOR vector;
		int vector_level = v->coordinate_system->parent_level;
		int target_level = target_coordinates->parent_level;

		//Consistency checks
		//EVDS_ASSERT(vector_level != target_level); //FIXME: verify when I'm not sleepy
		EVDS_ASSERT(vector_level >= 0);
		EVDS_ASSERT(target_level >= 0);

		//1. Move vector up until it reaches level of target coordinates
		EVDS_Vector_Copy(&vector,v);
		while (vector_level > target_level) {
			EVDS_Vector_ShortConvert(&vector,&vector,vector.coordinate_system->parent);
			vector_level = vector.coordinate_system->parent_level;
		}
		//Early exit: current coordinates ARE target coordinates
		if (vector.coordinate_system == target_coordinates) {
			EVDS_Vector_Copy(target,&vector);
			return;
		}

		//2. Move target coordinates up until they reach level of vector coordinates
		parent_track[parent_depth++] = target_coordinates;
		while (target_level > vector_level) {
			target_coordinates = target_coordinates->parent;
			parent_track[parent_depth++] = target_coordinates;
			target_level = target_coordinates->parent_level;
		}

		//3. Move both up until common parent is found. At this point either both 
		//parents are defined (levels are not zero), or both parents are null
		//(levels are zero).
		EVDS_ASSERT((target_coordinates->parent && vector.coordinate_system->parent) ||
					((!target_coordinates->parent) && (!vector.coordinate_system->parent)));
		while (target_coordinates->parent != vector.coordinate_system->parent) {
			//Move vector_level up
			EVDS_Vector_ShortConvert(&vector,&vector,vector.coordinate_system->parent);
			vector_level = vector.coordinate_system->parent_level;

			//Move target_level up
			parent_track[parent_depth++] = target_coordinates;
			target_coordinates = target_coordinates->parent;
			target_level = target_coordinates->parent_level;
		}

		//4. They have same parent now. Start tracing path back
		EVDS_ASSERT(target_coordinates->parent == vector.coordinate_system->parent);
		if (target_coordinates->parent) {
			EVDS_Vector_ShortConvert(&vector,&vector,vector.coordinate_system->parent);
		} else { //No parent must mean they are on the same level
			parent_depth--;
			EVDS_ASSERT(vector.coordinate_system == parent_track[parent_depth]);
		}
		while (parent_depth > 0) {
			parent_depth--;
			EVDS_Vector_ShortConvert(&vector,&vector,parent_track[parent_depth]);
		}

		//Return vector
		EVDS_Vector_Copy(target,&vector);
	}
}




////////////////////////////////////////////////////////////////////////////////
/// @brief Convert quaternion to target coordinates (short conversion)
////////////////////////////////////////////////////////////////////////////////
void EVDS_Quaternion_ShortConvert(EVDS_QUATERNION* target, EVDS_QUATERNION* q, EVDS_OBJECT* target_coordinates) {
	int target_is_child = 0;
	//Child and coordinate systems
	EVDS_OBJECT* child_coordinates;
	EVDS_OBJECT* parent_coordinates;
	//Child state vector
	EVDS_STATE_VECTOR* child_state;
	//Vector coordinate system
	EVDS_OBJECT* quaternion_coordinates;
	quaternion_coordinates = q->coordinate_system;

	//Verify inputs are correct
	EVDS_ASSERT(
		(target_coordinates->parent == quaternion_coordinates) ||	//Target coordinates are child of vector coordinates
		(target_coordinates == quaternion_coordinates->parent)		//Target coordinates are a parent of vector coordinates
	);

	//Determine child and parent coordinate systems
	if (quaternion_coordinates == target_coordinates->parent) {
		parent_coordinates = quaternion_coordinates; //a
		child_coordinates = target_coordinates; //b
		target_is_child = 1;
	} else {
		parent_coordinates = target_coordinates; //a
		child_coordinates = quaternion_coordinates; //b
		target_is_child = 0;
	}

	//Get correct state vectors (differentiate between public and private state vector)
#ifndef EVDS_SINGLETHREADED
	if (SIMC_Thread_GetUniqueID() == child_coordinates->integrate_thread) {
		child_state = &child_coordinates->private_state;
	} else if (SIMC_Thread_GetUniqueID() == child_coordinates->render_thread) {
		child_state = &child_coordinates->render_state;
	} else {
		child_state = &child_coordinates->state;
	}
#endif

	//Assert consistency
	EVDS_ASSERT(parent_coordinates == child_coordinates->parent);
	EVDS_ASSERT(child_state->orientation.coordinate_system == parent_coordinates);

	if (!target_is_child) {
		//Rotate from child to parent
		EVDS_Quaternion_Multiply(target,&child_state->orientation,q);
	} else {
		//Rotate from parent to child
		EVDS_Quaternion_MultiplyConjugatedQ(target,&child_state->orientation,q);
	}

	//Set coordinate system in target vector
	target->coordinate_system = target_coordinates;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Convert quaternion to target coordinates.
///
/// See EVDS_Vector_Convert() for the traversal algorithm used to determine conversion
/// path between two coordinate systems.
///
/// @param[out] target Quaternion, where result will be written
/// @param[in] q Quaternion, that must be converted
/// @param[in] target_coordinates Target coordinates, to which the vector must be converted
/// @returns Quaternion in target coordinates
////////////////////////////////////////////////////////////////////////////////
void EVDS_Quaternion_Convert(EVDS_QUATERNION* target, EVDS_QUATERNION* q, EVDS_OBJECT* target_coordinates) {
	//Do not convert if already in correct coordinates
	if (target_coordinates == q->coordinate_system) {
		if (target != q) memcpy(target,q,sizeof(EVDS_QUATERNION));
		return;
	}

	//Execute short conversion (FIXME: share this code with vectors)
	if ((target_coordinates->parent == q->coordinate_system) ||
		(target_coordinates == q->coordinate_system->parent)) {
		EVDS_Quaternion_ShortConvert(target,q,target_coordinates);
	} else {
		//Used for back-tracking when target is deeper than vector
		EVDS_OBJECT* parent_track[EVDS_VECTOR_MAX_DEPTH];
		int parent_depth = 0;

		//Temporary vector and parent levels (at what level of tree vectors are)
		EVDS_QUATERNION quaternion;
		int vector_level = q->coordinate_system->parent_level;
		int target_level = target_coordinates->parent_level;

		//Consistency checks
		//EVDS_ASSERT(vector_level != target_level); //FIXME: verify when I'm not sleepy
		EVDS_ASSERT(vector_level >= 0);
		EVDS_ASSERT(target_level >= 0);

		//1. Move vector up until it reaches level of target coordinates
		EVDS_Quaternion_Copy(&quaternion,q);
		while (vector_level > target_level) {
			EVDS_Quaternion_ShortConvert(&quaternion,&quaternion,quaternion.coordinate_system->parent);
			vector_level = quaternion.coordinate_system->parent_level;
		}
		//Early exit: current coordinates ARE target coordinates
		if (quaternion.coordinate_system == target_coordinates) {
			EVDS_Quaternion_Copy(target,&quaternion);
			return;
		}

		//2. Move target coordinates up until they reach level of vector coordinates
		parent_track[parent_depth++] = target_coordinates;
		while (target_level > vector_level) {
			target_coordinates = target_coordinates->parent;
			parent_track[parent_depth++] = target_coordinates;
			target_level = target_coordinates->parent_level;
		}

		//3. Move both up until common parent is found. At this point either both 
		//parents are defined (levels are not zero), or both parents are null
		//(levels are zero).
		EVDS_ASSERT((target_coordinates->parent && quaternion.coordinate_system->parent) ||
					((!target_coordinates->parent) && (!quaternion.coordinate_system->parent)));
		while (target_coordinates->parent != quaternion.coordinate_system->parent) {
			//Move vector_level up
			EVDS_Quaternion_ShortConvert(&quaternion,&quaternion,quaternion.coordinate_system->parent);
			vector_level = quaternion.coordinate_system->parent_level;

			//Move target_level up
			parent_track[parent_depth++] = target_coordinates;
			target_coordinates = target_coordinates->parent;
			target_level = target_coordinates->parent_level;
		}

		//4. They have same parent now. Start tracing path back
		EVDS_ASSERT(target_coordinates->parent == quaternion.coordinate_system->parent);
		if (target_coordinates->parent) {
			EVDS_Quaternion_ShortConvert(&quaternion,&quaternion,quaternion.coordinate_system->parent);
		} else { //No parent must mean they are on the same level
			parent_depth--;
			EVDS_ASSERT(quaternion.coordinate_system == parent_track[parent_depth]);
		}
		while (parent_depth > 0) {
			parent_depth--;
			EVDS_Quaternion_ShortConvert(&quaternion,&quaternion,parent_track[parent_depth]);
		}

		//Return vector
		EVDS_Quaternion_Copy(target,&quaternion);
	}
}




////////////////////////////////////////////////////////////////////////////////
/// @brief Gets components of the given vector in target coordinates.
///
/// @param[in] v Vector, whose coordinates must be obtained
/// @param[out] x X coordinate of vector in target coordinates
/// @param[out] y Y coordinate of vector in target coordinates
/// @param[out] z Z coordinate of vector in target coordinates
/// @param[in] target_coordinates Object, in whose coordinates the vector components must be returned
////////////////////////////////////////////////////////////////////////////////
void EVDS_Vector_Get(EVDS_VECTOR* v, EVDS_REAL* x, EVDS_REAL* y, EVDS_REAL* z, EVDS_OBJECT* target_coordinates) {
	EVDS_VECTOR temporary;
	EVDS_Vector_Convert(&temporary,v,target_coordinates);
	*x = temporary.x;
	*y = temporary.y;
	*z = temporary.z;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Sets components of the vector in target coordinates. Sets vector position and velocity to indefinite.
///
/// @param[out] target Vector which will be set
/// @param[in] derivative_level Vector type (position, acceleration, etc)
/// @param[in] target_coordinates Object, in whose coordinates the vector components must be returned
/// @param[in] x X coordinate of vector in target coordinates
/// @param[in] y Y coordinate of vector in target coordinates
/// @param[in] z Z coordinate of vector in target coordinates
////////////////////////////////////////////////////////////////////////////////
void EVDS_Vector_Set(EVDS_VECTOR* target, int derivative_level, EVDS_OBJECT* target_coordinates, EVDS_REAL x, EVDS_REAL y, EVDS_REAL z) {
	target->x = x;
	target->y = y;
	target->z = z;
	target->coordinate_system = target_coordinates;
	target->derivative_level = derivative_level;
	
	target->px = 0;
	target->py = 0;
	target->pz = 0;
	target->pcoordinate_system = 0;
	
	target->vx = 0;
	target->vy = 0;
	target->vz = 0;
	target->vcoordinate_system = 0;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Sets position of the vector.
///
/// @param[out] target Vector, position of which will be set
/// @param[in] target_coordinates Object, in whose coordinates the vector positions components must be returned
/// @param[in] x X coordinate of vectors position in target coordinates
/// @param[in] y Y coordinate of vectors position in target coordinates
/// @param[in] z Z coordinate of vectors position in target coordinates
////////////////////////////////////////////////////////////////////////////////
void EVDS_Vector_SetPosition(EVDS_VECTOR* target, EVDS_OBJECT* target_coordinates, EVDS_REAL x, EVDS_REAL y, EVDS_REAL z) {
	target->px = x;
	target->py = y;
	target->pz = z;
	target->pcoordinate_system = target_coordinates;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Gets position of the vector as a vector.
///
/// @param[in] v Vector, position of which must be determined
/// @param[out] position Vector of EVDS_VECTOR_POSITION type, in which vector "v" is located
////////////////////////////////////////////////////////////////////////////////
void EVDS_Vector_GetPositionVector(EVDS_VECTOR* v, EVDS_VECTOR* position) {
	position->x = v->px;
	position->y = v->py;
	position->z = v->pz;
	position->derivative_level = EVDS_VECTOR_POSITION;
	position->coordinate_system = v->pcoordinate_system;
	position->pcoordinate_system = 0;
	position->vcoordinate_system = 0;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Sets vector position by a position vector.
///
/// @param[in] v Vector, position of which must be set
/// @param[in] position Vector of EVDS_VECTOR_POSITION type, in which vector "v" is located
////////////////////////////////////////////////////////////////////////////////
void EVDS_Vector_SetPositionVector(EVDS_VECTOR* v, EVDS_VECTOR* position) {
	if (position) {
		EVDS_ASSERT(position->derivative_level == EVDS_VECTOR_POSITION);
		v->px = position->x;
		v->py = position->y;
		v->pz = position->z;
		v->pcoordinate_system = position->coordinate_system;
	} else {
		v->pcoordinate_system = 0;
	}
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Sets velocity of the vector
////////////////////////////////////////////////////////////////////////////////
void EVDS_Vector_SetVelocity(EVDS_VECTOR* target, EVDS_OBJECT* target_coordinates, EVDS_REAL x, EVDS_REAL y, EVDS_REAL z) {
	target->vx = x;
	target->vy = y;
	target->vz = z;
	target->vcoordinate_system = target_coordinates;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Gets velocity of the vector as a vector
////////////////////////////////////////////////////////////////////////////////
void EVDS_Vector_GetVelocityVector(EVDS_VECTOR* v, EVDS_VECTOR* velocity) {
	velocity->x = v->vx;
	velocity->y = v->vy;
	velocity->z = v->vz;
	velocity->derivative_level = EVDS_VECTOR_VELOCITY;
	velocity->coordinate_system = v->vcoordinate_system;
	velocity->pcoordinate_system = 0;
	velocity->vcoordinate_system = 0;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Sets vector velocity by a velocity vector
////////////////////////////////////////////////////////////////////////////////
void EVDS_Vector_SetVelocityVector(EVDS_VECTOR* v, EVDS_VECTOR* velocity) {
	if (velocity) {
		EVDS_ASSERT(velocity->derivative_level == EVDS_VECTOR_VELOCITY);
		v->vx = velocity->x;
		v->vy = velocity->y;
		v->vz = velocity->z;
		v->vcoordinate_system = velocity->coordinate_system;
	} else {
		v->vcoordinate_system = 0;
	}
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Change position where force is applied
////////////////////////////////////////////////////////////////////////////////
void EVDS_Vector_MoveForceToPosition(EVDS_VECTOR* force, EVDS_VECTOR* torque, EVDS_VECTOR* new_position) {
	EVDS_VECTOR force_position;
	EVDS_ASSERT(force->derivative_level == EVDS_VECTOR_FORCE);
	EVDS_ASSERT(new_position->derivative_level == EVDS_VECTOR_POSITION);

	//Move force into target position
	EVDS_Vector_GetPositionVector(force, &force_position); //Get force position
	if (!force_position.coordinate_system) EVDS_Vector_Copy(&force_position, new_position);
	EVDS_Vector_SetPositionVector(force, new_position); //Put force into center of mass
	EVDS_Vector_Subtract(&force_position, &force_position, new_position); //Find moment arm

	//Compute torque relative to center of mass
	EVDS_Vector_Cross(torque, &force_position, force);
	EVDS_Vector_SetPositionVector(torque, new_position);

	//Make sure this returns correct vector
	torque->pcoordinate_system = 0;
	torque->vcoordinate_system = 0;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Change position where torque is applied
////////////////////////////////////////////////////////////////////////////////
void EVDS_Vector_MoveTorqueToPosition(EVDS_VECTOR* force, EVDS_VECTOR* torque, EVDS_VECTOR* new_position) {
	EVDS_VECTOR torque_position;
	EVDS_ASSERT(torque->derivative_level == EVDS_VECTOR_TORQUE);
	EVDS_ASSERT(new_position->derivative_level == EVDS_VECTOR_POSITION);

	//Move torque into target position
	EVDS_Vector_GetPositionVector(torque, &torque_position); //Get force position
	if (!torque_position.coordinate_system) EVDS_Vector_Copy(&torque_position, new_position);
	EVDS_Vector_SetPositionVector(torque, new_position); //Put force into center of mass
	EVDS_Vector_Subtract(&torque_position, &torque_position, new_position); //Find moment arm

	//Compute torque relative to center of mass
	EVDS_Vector_Cross(force, torque, &torque_position);
	EVDS_Vector_SetPositionVector(force, new_position);

	//Make sure this returns correct vector (FIXME apply this to all operations!)
	force->pcoordinate_system = 0;
	force->vcoordinate_system = 0;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Nullify the vector velocity and position
///
/// This function will set position and velocity vectors of the target vector to zero
////////////////////////////////////////////////////////////////////////////////
void EVDS_Vector_NullifyPositionAndVelocity(EVDS_VECTOR* v) {
	//FIXME
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Copies data from source vector to target vector.
///
/// @param[out] target Target vector
/// @param[in] v Vector that will be copied to target
/// @returns Copy of the source vector
////////////////////////////////////////////////////////////////////////////////
void EVDS_Vector_Copy(EVDS_VECTOR* target, EVDS_VECTOR* v) {
	if (v == target) return;
	memcpy(target,v,sizeof(EVDS_VECTOR));
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Copies data from source quaternion to target quaternion.
///
/// @param[out] target Target quaternion
/// @param[in] q Quaternion that will be copied to target
/// @returns Copy of the source quaternion
////////////////////////////////////////////////////////////////////////////////
void EVDS_Quaternion_Copy(EVDS_QUATERNION* target, EVDS_QUATERNION* q) {
	if (q == target) return;
	memcpy(target,q,sizeof(EVDS_QUATERNION));
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Copies data from source state vector to target state vector.
///
/// @param[out] target Target state vector
/// @param[in] v State vector that will be copied to target
/// @returns Copy of the source state vector
////////////////////////////////////////////////////////////////////////////////
void EVDS_StateVector_Copy(EVDS_STATE_VECTOR* target, EVDS_STATE_VECTOR* v) {
	if (v == target) return;
	memcpy(target,v,sizeof(EVDS_STATE_VECTOR));
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Copies data from source state vector derivative to target state vector derivative.
///
/// @param[out] target Target state vector derivative
/// @param[in] v State vector derivative that will be copied to target
/// @returns Copy of the source state vector derivative
////////////////////////////////////////////////////////////////////////////////
void EVDS_StateVector_Derivative_Copy(EVDS_STATE_VECTOR_DERIVATIVE* target, EVDS_STATE_VECTOR_DERIVATIVE* v) {
	if (v == target) return;
	memcpy(target,v,sizeof(EVDS_STATE_VECTOR_DERIVATIVE));
}




////////////////////////////////////////////////////////////////////////////////
/// @brief Add vector v1 to vector v2
///
/// This operation returns sum of two vectors in the coordinate system of first vector (v1).
///
/// @param[in] v1 First vector
/// @param[in] v2 Second vector
/// @param[out] target \f$v1 + v2\f$
////////////////////////////////////////////////////////////////////////////////
void EVDS_Vector_Add(EVDS_VECTOR* target, EVDS_VECTOR* v1, EVDS_VECTOR* v2) {
	EVDS_VECTOR v21;
	EVDS_ASSERT((v1->derivative_level == v2->derivative_level) || 
		((v1->derivative_level == EVDS_VECTOR_POSITION) && (v2->derivative_level == EVDS_VECTOR_DISPLACEMENT)));
	EVDS_Vector_Convert(&v21,v2,v1->coordinate_system);

	target->x = v1->x + v21.x;
	target->y = v1->y + v21.y;
	target->z = v1->z + v21.z;
	target->coordinate_system = v1->coordinate_system;
	target->derivative_level = v1->derivative_level;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Subtract vector v2 from vector v1
///
/// This operation returns difference between two vectors in the coordinate system of first vector (v1).
///
/// @param[in] v1 First vector
/// @param[in] v2 Second vector
/// @param[out] target \f$v1 - v2\f$
////////////////////////////////////////////////////////////////////////////////
void EVDS_Vector_Subtract(EVDS_VECTOR* target, EVDS_VECTOR* v1, EVDS_VECTOR* v2) {
	EVDS_VECTOR v21;
	EVDS_ASSERT((v1->derivative_level == v2->derivative_level) ||
		((v1->derivative_level == EVDS_VECTOR_POSITION) && (v2->derivative_level == EVDS_VECTOR_DISPLACEMENT)));
	EVDS_Vector_Convert(&v21,v2,v1->coordinate_system);

	target->x = v1->x - v21.x;
	target->y = v1->y - v21.y;
	target->z = v1->z - v21.z;
	target->coordinate_system = v1->coordinate_system;
	target->derivative_level = v1->derivative_level;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Finds cross-product between vectors v1 and v2
////////////////////////////////////////////////////////////////////////////////
void EVDS_Vector_Cross(EVDS_VECTOR* target, EVDS_VECTOR* v1, EVDS_VECTOR* v2) {
	EVDS_REAL x,y,z;
	//EVDS_ASSERT(v1->derivative_level == v2->derivative_level);
	//EVDS_Vector_Move(v2,v2,v1);
	EVDS_VECTOR v21;
	EVDS_Vector_Convert(&v21,v2,v1->coordinate_system);

	x = v1->y*v21.z - v1->z*v21.y;
	y = v1->z*v21.x - v1->x*v21.z;
	z = v1->x*v21.y - v1->y*v21.x;
	target->x = x;
	target->y = y;
	target->z = z;
	target->coordinate_system = v1->coordinate_system;
	//target->derivative_level = v1->derivative_level;

	//Special case: "w x r" creates velocity vector
	if (((v1->derivative_level == EVDS_VECTOR_ANGULAR_VELOCITY) && (v2->derivative_level == EVDS_VECTOR_POSITION)) ||
		((v1->derivative_level == EVDS_VECTOR_POSITION) && (v2->derivative_level == EVDS_VECTOR_ANGULAR_VELOCITY))) {
		target->derivative_level = EVDS_VECTOR_VELOCITY;
	} else

	//Special case: "w x v" creates acceleration vector
	if (((v1->derivative_level == EVDS_VECTOR_ANGULAR_VELOCITY) && (v2->derivative_level == EVDS_VECTOR_VELOCITY)) ||
		((v1->derivative_level == EVDS_VECTOR_VELOCITY) && (v2->derivative_level == EVDS_VECTOR_ANGULAR_VELOCITY))) {
		target->derivative_level = EVDS_VECTOR_ACCELERATION;
	} else

	//Special case: "w' x r" creates acceleration vector
	if (((v1->derivative_level == EVDS_VECTOR_ANGULAR_ACCELERATION) && (v2->derivative_level == EVDS_VECTOR_POSITION)) ||
		((v1->derivative_level == EVDS_VECTOR_POSITION) && (v2->derivative_level == EVDS_VECTOR_ANGULAR_ACCELERATION))) {
		target->derivative_level = EVDS_VECTOR_ACCELERATION;
	} else

	//Special case: "f x r" creates torque vector
	if (((v1->derivative_level == EVDS_VECTOR_FORCE) && (v2->derivative_level == EVDS_VECTOR_POSITION)) ||
		((v1->derivative_level == EVDS_VECTOR_POSITION) && (v2->derivative_level == EVDS_VECTOR_FORCE))) {
		target->derivative_level = EVDS_VECTOR_TORQUE;
	} else

	//Special case: "t x r" creates force vector
	if (((v1->derivative_level == EVDS_VECTOR_TORQUE) && (v2->derivative_level == EVDS_VECTOR_POSITION)) ||
		((v1->derivative_level == EVDS_VECTOR_POSITION) && (v2->derivative_level == EVDS_VECTOR_TORQUE))) {
		target->derivative_level = EVDS_VECTOR_FORCE;
	} else

	//Default case
	{
		target->derivative_level = v1->derivative_level;
	}
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Find dot product between vector v1 and v2 in coordinate system of vector v1
///
/// @param[in] v1 First vector
/// @param[in] v2 Second vector
/// @param[out] target \f$v1 \cdot v2\f$
////////////////////////////////////////////////////////////////////////////////
void EVDS_Vector_Dot(EVDS_REAL* target, EVDS_VECTOR* v1, EVDS_VECTOR* v2) {
	EVDS_VECTOR v21;
	EVDS_ASSERT(v1->derivative_level == v2->derivative_level);
	EVDS_Vector_Convert(&v21,v2,v1->coordinate_system);

	*target = v1->x*v21.x+v1->y*v21.y+v1->z*v21.z;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Returns a normalized direction vector of v
////////////////////////////////////////////////////////////////////////////////
void EVDS_Vector_Normalize(EVDS_VECTOR* target, EVDS_VECTOR* v) {
	EVDS_REAL mag = sqrt(v->x*v->x + v->y*v->y + v->z*v->z);
	if (mag == 0.0) {
		target->x = 0.0;
		target->y = 0.0;
		target->z = 0.0;
	} else {
		target->x = v->x/mag;
		target->y = v->y/mag;
		target->z = v->z/mag;
	}
	target->coordinate_system = v->coordinate_system;
	target->derivative_level = EVDS_VECTOR_DIRECTION;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Return vector length
////////////////////////////////////////////////////////////////////////////////
void EVDS_Vector_Length(EVDS_REAL* target, EVDS_VECTOR* v) {
	*target = sqrt(v->x*v->x + v->y*v->y + v->z*v->z);
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Return distance between two vectors
////////////////////////////////////////////////////////////////////////////////
void EVDS_Vector_Distance(EVDS_REAL* target, EVDS_VECTOR* v1, EVDS_VECTOR* v2)
{
	EVDS_VECTOR v21;
	EVDS_ASSERT(v1->derivative_level == v2->derivative_level);
	EVDS_Vector_Convert(&v21, v2, v1->coordinate_system);

	*target = sqrt(
		(v1->x - v21.x)*(v1->x - v21.x) +
		(v1->y - v21.y)*(v1->y - v21.y) +
		(v1->z - v21.z)*(v1->z - v21.z));
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Check if two vectors are equal to eachother
////////////////////////////////////////////////////////////////////////////////
int EVDS_Vector_Equal(EVDS_VECTOR* v1, EVDS_VECTOR* v2) {
	EVDS_ASSERT(v1->derivative_level == v2->derivative_level);
	EVDS_REAL distance2 = 
		(v1->x - v2->x)*(v1->x - v2->x) +
		(v1->y - v2->y)*(v1->y - v2->y) +
		(v1->z - v2->z)*(v1->z - v2->z);
	return distance2 < EVDS_EPSf;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Multiply vector v by a scalar
////////////////////////////////////////////////////////////////////////////////
void EVDS_Vector_Multiply(EVDS_VECTOR* target, EVDS_VECTOR* v, EVDS_REAL scalar) {
	target->x = v->x*scalar;
	target->y = v->y*scalar;
	target->z = v->z*scalar;
	target->coordinate_system = v->coordinate_system;
	target->derivative_level = v->derivative_level;

	target->px = v->px; target->py = v->py; target->pz = v->pz;
	target->pcoordinate_system = v->pcoordinate_system;
	target->vx = v->vx; target->vy = v->vy; target->vz = v->vz;
	target->vcoordinate_system = v->vcoordinate_system;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Multiply vector v by a scalar and add it to source vector
////////////////////////////////////////////////////////////////////////////////
void EVDS_Vector_MultiplyAndAdd(EVDS_VECTOR* target, EVDS_VECTOR* source, EVDS_VECTOR* v, EVDS_REAL scalar) {
	EVDS_VECTOR temporary;
	EVDS_Vector_Initialize(temporary);

	temporary.x = v->x*scalar;
	temporary.y = v->y*scalar;
	temporary.z = v->z*scalar;
	temporary.coordinate_system = v->coordinate_system;
	temporary.derivative_level = v->derivative_level;

	EVDS_ASSERT(source->derivative_level == temporary.derivative_level);
	EVDS_Vector_Convert(&temporary,&temporary,source->coordinate_system);
	EVDS_ASSERT(source->coordinate_system == temporary.coordinate_system);

	target->x = source->x + temporary.x;
	target->y = source->y + temporary.y;
	target->z = source->z + temporary.z;
	target->coordinate_system = source->coordinate_system;
	target->derivative_level = source->derivative_level;
	target->pcoordinate_system = 0;
	target->vcoordinate_system = 0;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Multiply vector by a change in time and add it to the source vector
////////////////////////////////////////////////////////////////////////////////
void EVDS_Vector_MultiplyByTimeAndAdd(EVDS_VECTOR* target, EVDS_VECTOR* source, EVDS_VECTOR* v, EVDS_REAL delta_time) {
	EVDS_VECTOR temporary;

	temporary.x = v->x*delta_time;
	temporary.y = v->y*delta_time;
	temporary.z = v->z*delta_time;
	temporary.coordinate_system = v->coordinate_system;
	temporary.derivative_level = v->derivative_level;

	switch (temporary.derivative_level) {
		case EVDS_VECTOR_VELOCITY: 
			temporary.derivative_level = EVDS_VECTOR_POSITION; 
		break;
		case EVDS_VECTOR_ACCELERATION: 
			temporary.derivative_level = EVDS_VECTOR_VELOCITY; 
		break;
		case EVDS_VECTOR_ANGULAR_ACCELERATION: 
			temporary.derivative_level = EVDS_VECTOR_ANGULAR_VELOCITY; 
		break;
	}

	EVDS_ASSERT(source->derivative_level == temporary.derivative_level);
	EVDS_Vector_Convert(&temporary,&temporary,source->coordinate_system);
	EVDS_ASSERT(source->coordinate_system == temporary.coordinate_system);

	target->x = source->x + temporary.x;
	target->y = source->y + temporary.y;
	target->z = source->z + temporary.z;
	target->coordinate_system = source->coordinate_system;
	target->derivative_level = source->derivative_level;
	target->pcoordinate_system = 0;
	target->vcoordinate_system = 0;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Interpolate vector linearly between two vectors.
////////////////////////////////////////////////////////////////////////////////
void EVDS_Vector_Interpolate(EVDS_VECTOR* target, EVDS_VECTOR* v1, EVDS_VECTOR* v2, EVDS_REAL t) {
	EVDS_VECTOR temporary;
	EVDS_Vector_Multiply(&temporary,v1,1-t);
	EVDS_Vector_MultiplyAndAdd(&temporary,&temporary,v2,t);
	EVDS_Vector_Copy(target,&temporary);
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Multiply state vector derivative v by a scalar and add it to source state vector derivative
///
/// \f{eqnarray*}{
///		\left[ \begin{array}{c} 
///			\bar v \\
///			\bar a \\
///			\bar \omega \\
///			\bar \alpha \\
///			\bar F \\
///			\bar T
///		\end{array} \right]_{target} =
///
///		\left[ \begin{array}{c} 
///			\bar v \\
///			\bar a \\
///			\bar \omega \\
///			\bar \alpha \\
///			\bar F \\
///			\bar T
///		\end{array} \right]_{source} + scalar \cdot
///		\left[ \begin{array}{c} 
///			\bar v \\
///			\bar a \\
///			\bar \omega \\
///			\bar \alpha \\
///			\bar F \\
///			\bar T
///		\end{array} \right]_{v}
/// \f}
///
/// @param[out] target Result of the operation will be written here
/// @param[in] source State vector derivative
/// @param[in] v State vector derivative that will be multiplied and added to source
/// @param[in] scalar Scalar value
///
/// @returns New state vector derivative
////////////////////////////////////////////////////////////////////////////////
void EVDS_StateVector_Derivative_MultiplyAndAdd(EVDS_STATE_VECTOR_DERIVATIVE* target, EVDS_STATE_VECTOR_DERIVATIVE* source, 
												EVDS_STATE_VECTOR_DERIVATIVE* v, EVDS_REAL scalar) {
	EVDS_Vector_MultiplyAndAdd(&target->velocity,&source->velocity,&v->velocity,scalar);
	EVDS_Vector_MultiplyAndAdd(&target->acceleration,&source->acceleration,&v->acceleration,scalar);
	EVDS_Vector_MultiplyAndAdd(&target->angular_velocity,&source->angular_velocity,&v->angular_velocity,scalar);
	EVDS_Vector_MultiplyAndAdd(&target->angular_acceleration,&source->angular_acceleration,&v->angular_acceleration,scalar);
	EVDS_Vector_MultiplyAndAdd(&target->force,&source->force,&v->force,scalar);
	EVDS_Vector_MultiplyAndAdd(&target->torque,&source->torque,&v->torque,scalar);
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Multiply state vector derivative by a change in time and add it to the source state vector
///
/// Sets acceleration, angular acceleration in target state vector from the derivative. Adds
/// delta time to state vector time.
/// All the vectors are propagated using EVDS_Vector_MultiplyTimeAndAdd(), quaternions are
/// propagated from angular velocity vector using the quaternion derivative:
/// \f{eqnarray*}{
///		\dot{q}(t) &=& \frac{1}{2}\cdot [0, \omega(t)] \cdot q(t) \\
///		q'(t+\Delta t) &=& q(t) + \Delta t \cdot \dot{q}(t)
/// \f}
///
/// Expanding the quaternion multiplication, the final expression is:
/// \f{eqnarray*}{
///		q'(t+\Delta t) &=& q(t) + \Delta t \cdot \frac{1}{2}
///		\left[ \begin{array}{cccc} 
///			0        & - \omega_x & - \omega_y & - \omega_z \\ 
///			\omega_x &   0        &   \omega_z & - \omega_y \\ 
///			\omega_y & - \omega_z &   0        &   \omega_x \\ 
///			\omega_z &   \omega_y & - \omega_x &   0        \\ 
///		\end{array} \right]
///		\left[ \begin{array}{c} 
///			q_0 \\
///			q_1 \\
///			q_2 \\
///			q_3 \\
///		\end{array} \right] \\
///
///		&=& q(t) + \Delta t \cdot \frac{1}{2}
///		\left[ \begin{array}{c} 
///			q_0 \cdot 0        - q_1 \cdot \omega_x - q_2 \cdot \omega_y - q_3 \cdot \omega_z \\ 
///			q_0 \cdot \omega_x + q_1 \cdot 0        + q_2 \cdot \omega_z - q_3 \cdot \omega_y \\ 
///			q_0 \cdot \omega_y - q_1 \cdot \omega_z + q_2 \cdot 0        + q_3 \cdot \omega_x \\ 
///			q_0 \cdot \omega_z + q_1 \cdot \omega_y - q_2 \cdot \omega_x + q_3 \cdot 0        \\ 
///		\end{array} \right]
/// \f}
///
/// @param[out] target Propagated state vector (at \f$t = \Delta t\f$) will be written here
/// @param[in] source State vector at \f$t = 0\f$
/// @param[in] v State vector derivative at \f$t = 0\f$
/// @param[in] delta_time Time step
///
/// @returns Propagated state vector
////////////////////////////////////////////////////////////////////////////////
void EVDS_StateVector_MultiplyByTimeAndAdd(EVDS_STATE_VECTOR* target, EVDS_STATE_VECTOR* source, 
										   EVDS_STATE_VECTOR_DERIVATIVE* v, EVDS_REAL delta_time) {
	EVDS_QUATERNION delta_quaternion;
	EVDS_VECTOR axis;
	EVDS_REAL magnitude;
	if (target != source) memset(target,0,sizeof(EVDS_STATE_VECTOR)); //Terrible requirement, FIXME
	target->time = source->time + delta_time / 86400.0;

	EVDS_Vector_MultiplyByTimeAndAdd(&target->velocity,&source->velocity,&v->acceleration,delta_time);
	EVDS_Vector_MultiplyByTimeAndAdd(&target->position,&source->position,&v->velocity,delta_time);
	EVDS_Vector_Copy(&target->acceleration,&v->acceleration);

	EVDS_Vector_MultiplyByTimeAndAdd(&target->angular_velocity,&source->angular_velocity,&v->angular_acceleration,delta_time);
	EVDS_Vector_Copy(&target->angular_acceleration,&v->angular_acceleration);

	EVDS_ASSERT(source->orientation.coordinate_system == v->angular_velocity.coordinate_system);
	EVDS_ASSERT(source->orientation.coordinate_system == source->orientation.coordinate_system);

	EVDS_Vector_Length(&magnitude, &v->angular_velocity);
	EVDS_Vector_Normalize(&axis,&v->angular_velocity);
	EVDS_Quaternion_FromVectorAngle(&delta_quaternion,&axis,magnitude*delta_time);
	EVDS_Quaternion_Multiply(&target->orientation,&delta_quaternion,&source->orientation);

	EVDS_Quaternion_Normalize(&target->orientation,&target->orientation);
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Interpolate between two state vectors
////////////////////////////////////////////////////////////////////////////////
void EVDS_StateVector_Interpolate(EVDS_STATE_VECTOR* target, EVDS_STATE_VECTOR* v1, EVDS_STATE_VECTOR* v2, EVDS_REAL t) {
	if (t < 0.0) t = 0.0;
	if (t > 1.0) t = 1.0;

	EVDS_Vector_Interpolate(&target->velocity,&v1->velocity,&v2->velocity,t);
	EVDS_Vector_Interpolate(&target->position,&v1->position,&v2->position,t);
	EVDS_Vector_Interpolate(&target->acceleration,&v1->acceleration,&v2->acceleration,t);
	EVDS_Vector_Interpolate(&target->angular_velocity,&v1->angular_velocity,&v2->angular_velocity,t);
	EVDS_Vector_Interpolate(&target->angular_acceleration,&v1->angular_acceleration,&v2->angular_acceleration,t);
	EVDS_Quaternion_Interpolate(&target->orientation,&v1->orientation,&v2->orientation,t);
	//FIXME: use slerp

	EVDS_Quaternion_Normalize(&target->orientation,&target->orientation);
}




////////////////////////////////////////////////////////////////////////////////
/// @brief Set quaternion from euler angles
////////////////////////////////////////////////////////////////////////////////
void EVDS_Quaternion_FromEuler(EVDS_QUATERNION* target, EVDS_OBJECT* target_coordinates, EVDS_REAL x, EVDS_REAL y, EVDS_REAL z) {
	EVDS_REAL c1 = cos(x*0.5);
	EVDS_REAL c2 = cos(y*0.5);
	EVDS_REAL c3 = cos(z*0.5);

	EVDS_REAL s1 = sin(x*0.5);
	EVDS_REAL s2 = sin(y*0.5);
	EVDS_REAL s3 = sin(z*0.5);

	target->q[0] = c1*c2*c3 + s1*s2*s3;
	target->q[1] = s1*c2*c3 - c1*s2*s3;
	target->q[2] = c1*s2*c3 + s1*c2*s3;
	target->q[3] = c1*c2*s3 - s1*s2*c3;
	target->coordinate_system = target_coordinates;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Get quaternion from euler angles
////////////////////////////////////////////////////////////////////////////////
void EVDS_Quaternion_ToEuler(EVDS_QUATERNION* q, EVDS_OBJECT* target_coordinates, EVDS_REAL* x, EVDS_REAL* y, EVDS_REAL* z) {
	//      w  x  y  z
	EVDS_REAL q0,q1,q2,q3;
	EVDS_REAL sine;
	EVDS_QUATERNION temp;
	EVDS_Quaternion_Convert(&temp,q,target_coordinates);
	q0 = temp.q[0];	q1 = temp.q[1];	q2 = temp.q[2];	q3 = temp.q[3];

	sine = 2*(q0*q2 - q3*q1);
	if (sine > 1.0) sine = 1.0;
	if (sine < -1.0) sine = -1.0;

	if (x) *x = atan2(2*q0*q1+2*q2*q3, 1-2*q1*q1-2*q2*q2);
	if (y) *y = asin(sine);
	if (z) *z = atan2(2*q0*q3+2*q1*q2, 1-2*q2*q2-2*q3*q3);
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Set rotation vector and angle in target coordinate system
////////////////////////////////////////////////////////////////////////////////
void EVDS_Quaternion_FromVectorAngle(EVDS_QUATERNION* target, EVDS_VECTOR* axis, EVDS_REAL angle) {
	EVDS_REAL c = cos(angle*0.5);
	EVDS_REAL s = sin(angle*0.5);

	target->q[0] = c;
	target->q[1] = s * axis->x;
	target->q[2] = s * axis->y;
	target->q[3] = s * axis->z;
	target->coordinate_system = axis->coordinate_system;	
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Get rotation vector and angle in target coordinate system
////////////////////////////////////////////////////////////////////////////////
void EVDS_Quaternion_ToVectorAngle(EVDS_QUATERNION* q, EVDS_VECTOR* axis, EVDS_REAL* angle) {
	//      w  x  y  z
	EVDS_REAL q0,q1,q2,q3;
	EVDS_REAL mag;
	q0 = q->q[0];	q1 = q->q[1];	q2 = q->q[2];	q3 = q->q[3];

	//Calculate rotation axis (normalized x,y,z components of quaternion)
	EVDS_Vector_Set(axis,EVDS_VECTOR_DIRECTION,q->coordinate_system,q1,q2,q3);
	EVDS_Vector_Normalize(axis,axis);

	//Find arccos of real component of a corresponding normalized quaternion
	mag = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	if (mag != 0.0) {
		*angle = 2.0*acos(q0/mag);
	} else {
		*angle = 0.0;
	}
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Return quaternion as a 4x4 matrix
////////////////////////////////////////////////////////////////////////////////
void EVDS_Quaternion_ToMatrix(EVDS_QUATERNION* q, EVDS_MATRIX m)
{
	//      w  x  y  z
	EVDS_REAL q0,q1,q2,q3;
	q0 = q->q[0]; q1 = q->q[1]; q2 = q->q[2]; q3 = q->q[3];

	m[0*4+0] = q0*q0+q1*q1-q2*q2-q3*q3;
	m[0*4+1] = 2*q1*q2 + 2*q0*q3;
	m[0*4+2] = 2*q1*q3 - 2*q0*q2;
	m[0*4+3] = 0;

	m[1*4+0] = 2*q1*q2 - 2*q0*q3;
	m[1*4+1] = q0*q0-q1*q1+q2*q2-q3*q3;
	m[1*4+2] = 2*q2*q3 + 2*q0*q1;
	m[1*4+3] = 0;

	m[2*4+0] = 2*q1*q3 + 2*q0*q2;
	m[2*4+1] = 2*q2*q3 - 2*q0*q1;
	m[2*4+2] = q0*q0-q1*q1-q2*q2+q3*q3;
	m[2*4+3] = 0;

	m[3*4+0] = 0;
	m[3*4+1] = 0;
	m[3*4+2] = 0;
	m[3*4+3] = q0*q0+q1*q1+q2*q2+q3*q3;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Multiply quaternion \f$q\f$ by \f$r\f$.
///
/// This operation rotates attitude specified by quaternion \f$r\f$ by quaternion \f$q\f$.
/// For example:
/// ~~~{.c}
///		EVDS_QUATERNION attitude;
///		EVDS_QUATERNION delta;
///		EVDS_Quaternion_Multiply(&attitude,&delta,&attitude); //Rotate attitude by delta
/// ~~~
///
/// The operation requires both quaternions to be in same coordinate system, or quaternion
/// \f$q\f$ must be specified in parent coordinates of \f$r\f$. In the latter case the
/// operation is equal to transformation of quaternion \f$r\f$ in child coordinates to
/// parent coordinates (child coordinates rotated in parent coordinates by \f$q\f$).
///
/// The quaternion multiplication operation is defined as following:
/// \f{eqnarray*}{
///		q \cdot r &=&
///		\left[ \begin{array}{c} 
///			q_0 \\
///			q_1 \\
///			q_2 \\
///			q_3 \\
///		\end{array} \right] \cdot
///		\left[ \begin{array}{c} 
///			r_0 \\
///			r_1 \\
///			r_2 \\
///			r_3 \\
///		\end{array} \right]
///		&=&
///		\left[ \begin{array}{c} 
///			r_0 q_0 - r_1 q_1 - r_2 q_2 - r_3 q_3 \\
///			r_0 q_1 + r_1 q_0 - r_2 q_3 + r_3 q_2 \\
///			r_0 q_2 + r_1 q_3 + r_2 q_0 - r_3 q_1 \\
///			r_0 q_3 - r_1 q_2 + r_2 q_1 + r_3 q_0 \\
///		\end{array} \right] \\
/// \f}
////////////////////////////////////////////////////////////////////////////////
void EVDS_Quaternion_Multiply(EVDS_QUATERNION* target, EVDS_QUATERNION* q, EVDS_QUATERNION* r)
{
	EVDS_REAL q0,q1,q2,q3;
	EVDS_REAL r0,r1,r2,r3;

	//Quaternion coordinate system conversion, unless trying to rotate quaternion
	//from one coordinate system into another.
	if ((q->coordinate_system != r->coordinate_system) &&
		(q->coordinate_system != r->coordinate_system->parent)) {
		EVDS_QUATERNION qt;
		EVDS_Quaternion_Convert(&qt,q,r->coordinate_system);
		q0 = qt.q[0]; q1 = qt.q[1]; q2 = qt.q[2]; q3 = qt.q[3];
	} else {
		q0 = q->q[0]; q1 = q->q[1]; q2 = q->q[2]; q3 = q->q[3];
	}
	r0 = r->q[0]; r1 = r->q[1]; r2 = r->q[2]; r3 = r->q[3];

	target->q[0] = r0 * q0 - r1 * q1 - r2 * q2 - r3 * q3;
	target->q[1] = r0 * q1 + r1 * q0 - r2 * q3 + r3 * q2;
	target->q[2] = r0 * q2 + r1 * q3 + r2 * q0 - r3 * q1;
	target->q[3] = r0 * q3 - r1 * q2 + r2 * q1 + r3 * q0;
	target->coordinate_system = r->coordinate_system;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Multiply conjugate of quaternion \f$q\f$ by \f$r\f$.
///
/// This operation rotates attitude specified by quaternion \f$r\f$ by opposite to quaternion \f$q\f$.
/// For example:
/// ~~~{.c}
///		EVDS_QUATERNION attitude;
///		EVDS_QUATERNION delta;
///		EVDS_Quaternion_Multiply(&attitude,&delta,&attitude); //Rotate attitude by delta
///		EVDS_Quaternion_MultiplyQ(&attitude,&delta,&attitude); //Undo the rotation
/// ~~~
///
/// The operation requires both quaternions to be in same coordinate system. This operation
/// can be used to transform quaternion from parent coordinates to child coordinates
/// (child coordinates rotated in parent coordinates by \f$q\f$).
///
/// This operation is defined as following:
/// \f{eqnarray*}{
///		q^{-1} \cdot r &=&
///		\left[ \begin{array}{c} 
///			q_0 \\
///			q_1 \\
///			q_2 \\
///			q_3 \\
///		\end{array} \right]^{-1} \cdot
///		\left[ \begin{array}{c} 
///			r_0 \\
///			r_1 \\
///			r_2 \\
///			r_3 \\
///		\end{array} \right]
///		&=&
///		\left[ \begin{array}{c} 
///			  r_0 q_0 + r_1 q_1 + r_2 q_2 + r_3 q_3 \\
///			- r_0 q_1 + r_1 q_0 + r_2 q_3 - r_3 q_2 \\
///			- r_0 q_2 - r_1 q_3 + r_2 q_0 + r_3 q_1 \\
///			- r_0 q_3 + r_1 q_2 - r_2 q_1 + r_3 q_0 \\
///		\end{array} \right] \\
/// \f}
////////////////////////////////////////////////////////////////////////////////
void EVDS_Quaternion_MultiplyConjugatedQ(EVDS_QUATERNION* target, EVDS_QUATERNION* q, EVDS_QUATERNION* r)
{
	EVDS_REAL q0,q1,q2,q3;
	EVDS_REAL r0,r1,r2,r3;

	//Quaternion coordinate system conversion
	if (q->coordinate_system != r->coordinate_system) {
		EVDS_QUATERNION qt;
		EVDS_Quaternion_Convert(&qt,q,r->coordinate_system);
		q0 = qt.q[0]; q1 = -qt.q[1]; q2 = -qt.q[2]; q3 = -qt.q[3];
	} else {
		q0 = q->q[0]; q1 = -q->q[1]; q2 = -q->q[2]; q3 = -q->q[3];
	}
	r0 = r->q[0]; r1 =  r->q[1]; r2 =  r->q[2]; r3 =  r->q[3];

	target->q[0] = r0 * q0 - r1 * q1 - r2 * q2 - r3 * q3;
	target->q[1] = r0 * q1 + r1 * q0 - r2 * q3 + r3 * q2;
	target->q[2] = r0 * q2 + r1 * q3 + r2 * q0 - r3 * q1;
	target->q[3] = r0 * q3 - r1 * q2 + r2 * q1 + r3 * q0;
	target->coordinate_system = r->coordinate_system;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Multiply quaternion \f$q\f$ by conjugate of \f$r\f$.
///
/// This operation provides a shortcut, as similar to EVDS_Quaternion_MultiplyConjugatedQ().
/// It is only provided for consistency, and defined as following:
/// \f{eqnarray*}{
///		q \cdot r^{-1} &=&
///		\left[ \begin{array}{c} 
///			q_0 \\
///			q_1 \\
///			q_2 \\
///			q_3 \\
///		\end{array} \right] \cdot
///		\left[ \begin{array}{c} 
///			r_0 \\
///			r_1 \\
///			r_2 \\
///			r_3 \\
///		\end{array} \right]^{-1}
///		&=&
///		\left[ \begin{array}{c} 
///			r_0 q_0 + r_1 q_1 + r_2 q_2 + r_3 q_3 \\
///			r_0 q_1 - r_1 q_0 + r_2 q_3 - r_3 q_2 \\
///			r_0 q_2 - r_1 q_3 - r_2 q_0 + r_3 q_1 \\
///			r_0 q_3 + r_1 q_2 - r_2 q_1 - r_3 q_0 \\
///		\end{array} \right] \\
/// \f}
////////////////////////////////////////////////////////////////////////////////
void EVDS_Quaternion_MultiplyConjugatedR(EVDS_QUATERNION* target, EVDS_QUATERNION* q, EVDS_QUATERNION* r)
{
	EVDS_REAL q0,q1,q2,q3;
	EVDS_REAL r0,r1,r2,r3;

	//Quaternion coordinate system conversion
	if (q->coordinate_system != r->coordinate_system) {
		EVDS_QUATERNION qt;
		EVDS_Quaternion_Convert(&qt,q,r->coordinate_system);
		q0 = qt.q[0]; q1 =  qt.q[1]; q2 =  qt.q[2]; q3 =  qt.q[3];
	} else {
		q0 = q->q[0]; q1 =  q->q[1]; q2 =  q->q[2]; q3 =  q->q[3];
	}
	r0 = r->q[0]; r1 = -r->q[1]; r2 = -r->q[2]; r3 = -r->q[3];

	target->q[0] = r0 * q0 - r1 * q1 - r2 * q2 - r3 * q3;
	target->q[1] = r0 * q1 + r1 * q0 - r2 * q3 + r3 * q2;
	target->q[2] = r0 * q2 + r1 * q3 + r2 * q0 - r3 * q1;
	target->q[3] = r0 * q3 - r1 * q2 + r2 * q1 + r3 * q0;
	target->coordinate_system = r->coordinate_system;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Multiply quaternion by a scalar.
///
/// @note This operation will most likely change quaternions length. Be careful with operations
///       which require a normalized quaternion.
///
/// It is defined as:
/// \f{eqnarray*}{
///		q \cdot scalar &=&
///		\left[ \begin{array}{c} 
///			q_0 \cdot scalar \\
///			q_1 \cdot scalar \\
///			q_2 \cdot scalar \\
///			q_3 \cdot scalar \\
///		\end{array} \right]
/// \f}
////////////////////////////////////////////////////////////////////////////////
void EVDS_Quaternion_MultiplyScalar(EVDS_QUATERNION* target, EVDS_QUATERNION* source, EVDS_REAL scalar) {
	target->q[0] = source->q[0]*scalar;
	target->q[1] = source->q[1]*scalar;
	target->q[2] = source->q[2]*scalar;
	target->q[3] = source->q[3]*scalar;
	target->coordinate_system = source->coordinate_system;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Spherical quaternion interpolation.
////////////////////////////////////////////////////////////////////////////////
void EVDS_Quaternion_Interpolate(EVDS_QUATERNION* target, EVDS_QUATERNION* q1, EVDS_QUATERNION* q2, EVDS_REAL t) {
	//FIXME
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Return normalized quaternion
////////////////////////////////////////////////////////////////////////////////
void EVDS_Quaternion_Normalize(EVDS_QUATERNION* target, EVDS_QUATERNION* q) {
	EVDS_REAL q0,q1,q2,q3;
	EVDS_REAL qmag;

	q0 = q->q[0]; q1 = q->q[1]; q2 = q->q[2]; q3 = q->q[3];
	qmag = sqrt(q0*q0+q1*q1+q2*q2+q3*q3);
	if (qmag == 0.0) qmag = 1.0;

	target->q[0] = q0/qmag;
	target->q[1] = q1/qmag;
	target->q[2] = q2/qmag;
	target->q[3] = q3/qmag;
	target->coordinate_system = q->coordinate_system;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Rotate vector by quaternion q
////////////////////////////////////////////////////////////////////////////////
void EVDS_Vector_Rotate(EVDS_VECTOR* target, EVDS_VECTOR* v, EVDS_QUATERNION* q)
{
	EVDS_REAL q0,q1,q2,q3;
	EVDS_REAL    v1,v2,v3;
	EVDS_REAL t0,t1,t2,t3;

	//EVDS_Quaternion_Convert(v,v,v->coordinate_system->parent);
	//FIXME: Add proper coordinate conversion
	EVDS_ASSERT((v->coordinate_system == q->coordinate_system) ||
		(v->coordinate_system->parent == q->coordinate_system));

	              v1 = v->x;    v2 = v->y;    v3 = v->z;
	q0 = q->q[0]; q1 = q->q[1]; q2 = q->q[2]; q3 = q->q[3];

	//t = q * v
	t0 = /*v0 * q0*/ - v1 * q1 - v2 * q2 - v3 * q3;
	t1 = /*v0 * q1*/ + v1 * q0 - v2 * q3 + v3 * q2;
	t2 = /*v0 * q2*/ + v1 * q3 + v2 * q0 - v3 * q1;
	t3 = /*v0 * q3*/ - v1 * q2 + v2 * q1 + v3 * q0;

	//target = t * (q^-1)
	//target->w = q0 * t0 + q1 * t1 + q2 * t2 + q3 * t3;
	  target->x = q0 * t1 - q1 * t0 + q2 * t3 - q3 * t2;
	  target->y = q0 * t2 - q1 * t3 - q2 * t0 + q3 * t1;
	  target->z = q0 * t3 + q1 * t2 - q2 * t1 - q3 * t0;

	target->coordinate_system = q->coordinate_system;
	target->derivative_level = v->derivative_level;
}

//v must be in coordinate system child to quaternion
//resulting vector is in coordinate system of quaternion
////////////////////////////////////////////////////////////////////////////////
/// @brief Rotate vector by a conjugate of quaternion q
////////////////////////////////////////////////////////////////////////////////
void EVDS_Vector_RotateConjugated(EVDS_VECTOR* target, EVDS_VECTOR* v, EVDS_QUATERNION* q)
{
	EVDS_REAL q0,q1,q2,q3;
	EVDS_REAL    v1,v2,v3;
	EVDS_REAL t0,t1,t2,t3;

	//EVDS_Quaternion_Convert(q,q,v->coordinate_system);
	//EVDS_Vector_Convert(v,v,q->coordinate_system);
	//FIXME: add proper coordinate conversion
	EVDS_ASSERT((v->coordinate_system == q->coordinate_system) ||
		(v->coordinate_system->parent == q->coordinate_system));

	              v1 = v->x;     v2 = v->y;     v3 = v->z;
	q0 = q->q[0]; q1 = -q->q[1]; q2 = -q->q[2]; q3 = -q->q[3];

	//t = q * v
	t0 = /*v0 * q0*/ - v1 * q1 - v2 * q2 - v3 * q3;
	t1 = /*v0 * q1*/ + v1 * q0 - v2 * q3 + v3 * q2;
	t2 = /*v0 * q2*/ + v1 * q3 + v2 * q0 - v3 * q1;
	t3 = /*v0 * q3*/ - v1 * q2 + v2 * q1 + v3 * q0;

	//target = t * (q^-1)
	//target->w = q0 * t0 + q1 * t1 + q2 * t2 + q3 * t3;
	  target->x = q0 * t1 - q1 * t0 + q2 * t3 - q3 * t2;
	  target->y = q0 * t2 - q1 * t3 - q2 * t0 + q3 * t1;
	  target->z = q0 * t3 + q1 * t2 - q2 * t1 - q3 * t0;

	target->coordinate_system = q->coordinate_system;
	target->derivative_level = v->derivative_level;
}




////////////////////////////////////////////////////////////////////////////////
/// @brief Multiply 3x3 tensor built of vectors by a vector.
///
/// This operation returns vector, which is a result of the following operation:
/// \f[
///		target = \left[ \begin{array}{cccc} 
///			m_{xx} & m_{xy} & m_{xz} \\ 
///			m_{yx} & m_{yy} & m_{yz} \\ 
///			m_{zx} & m_{zy} & m_{zz}
///		\end{array} \right]
///		\left[ \begin{array}{cccc} 
///			v_{x} \\ 
///			v_{y} \\ 
///			v_{z}
///		\end{array} \right]
/// \f]
///
/// This operation can only be used if all tensor component coordinate systems are a child of
/// vectors coordinate system or have the same coordinate system.
///
/// The target vector will have derivative of the vector by which tensor is multiplied, and will be in
/// coordinates system of that vector.
///
/// @returns Vector multiplied by the matrix
////////////////////////////////////////////////////////////////////////////////
void EVDS_Tensor_MultiplyByVector(EVDS_VECTOR* target, EVDS_VECTOR* mx, EVDS_VECTOR* my, EVDS_VECTOR* mz, EVDS_VECTOR* v) {
	EVDS_ASSERT(mx->coordinate_system == v->coordinate_system);
	EVDS_ASSERT(my->coordinate_system == v->coordinate_system);
	EVDS_ASSERT(mz->coordinate_system == v->coordinate_system);

	target->x = mx->x * v->x + mx->y * v->y + mx->z * v->z;
	target->y = my->x * v->x + my->y * v->y + my->z * v->z;
	target->z = mz->x * v->x + mz->y * v->y + mz->z * v->z;
	target->coordinate_system = v->coordinate_system;
	target->derivative_level = v->derivative_level;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Rotate 3x3 tensor built out of vectors by a quaternion.
///
/// This operation returns tensor, which is a result of the following operation:
/// \f[
///		\left[ \begin{array}{cccc} 
///			t_{xx} & t_{xy} & t_{xz} \\ 
///			t_{yx} & t_{yy} & t_{yz} \\ 
///			t_{zx} & t_{zy} & t_{zz}
///		\end{array} \right]
///			=
///			Q_m \cdot m \cdot Q_m^t
///			=
///		\left[ \begin{array}{cccc} 
///			Q_{0} & Q_{1} & Q_{2} \\ 
///			Q_{4} & Q_{5} & Q_{6} \\ 
///			Q_{8} & Q_{9} & Q_{10}
///		\end{array} \right]
///		\left[ \begin{array}{cccc} 
///			m_{xx} & m_{xy} & m_{xz} \\ 
///			m_{yx} & m_{yy} & m_{yz} \\ 
///			m_{zx} & m_{zy} & m_{zz}
///		\end{array} \right]
///		\left[ \begin{array}{cccc} 
///			Q_{0} & Q_{4} & Q_{8} \\ 
///			Q_{1} & Q_{5} & Q_{9} \\ 
///			Q_{2} & Q_{6} & Q_{10}
///		\end{array} \right]
/// \f]
/// where \f$Q\f$ is the rotation matrix generated from quaternion (internally a 4x4 matrix).
/// The resulting tensor is returned in a coordinate system, which is rotated by quaternion.
///
/// This operation can only be used if all tensor component coordinate systems are a child of
/// quaternions coordinate system or have the same coordinate system.
///
/// The returned tensor components will be in coordinate system of quaternion, and have
/// derivative level of the source tensor components.
///
/// @returns Tensor rotated by the quaternion
////////////////////////////////////////////////////////////////////////////////
void EVDS_Tensor_Rotate(EVDS_VECTOR* tx, EVDS_VECTOR* ty, EVDS_VECTOR* tz,
						EVDS_VECTOR* mx, EVDS_VECTOR* my, EVDS_VECTOR* mz,
						EVDS_QUATERNION* q) {
	EVDS_VECTOR qx,qy,qz;
	EVDS_MATRIX Q;
	EVDS_ASSERT((mx->coordinate_system == q->coordinate_system) ||
		(mx->coordinate_system->parent == q->coordinate_system));
	EVDS_ASSERT((my->coordinate_system == q->coordinate_system) ||
		(my->coordinate_system->parent == q->coordinate_system));
	EVDS_ASSERT((mz->coordinate_system == q->coordinate_system) ||
		(mz->coordinate_system->parent == q->coordinate_system));

	EVDS_Vector_Initialize(*tx);
	EVDS_Vector_Initialize(*ty);
	EVDS_Vector_Initialize(*tz);

	EVDS_Quaternion_ToMatrix(q,Q);

	//q = Q * m
	qx.x  = mx->x * Q[0] + my->x * Q[1] + mz->x * Q[2];
	qy.x  = mx->x * Q[4] + my->x * Q[5] + mz->x * Q[6];
	qz.x  = mx->x * Q[8] + my->x * Q[9] + mz->x * Q[10];

	qx.y  = mx->y * Q[0] + my->y * Q[1] + mz->y * Q[2];
	qy.y  = mx->y * Q[4] + my->y * Q[5] + mz->y * Q[6];
	qz.y  = mx->y * Q[8] + my->y * Q[9] + mz->y * Q[10];

	qx.z  = mx->z * Q[0] + my->z * Q[1] + mz->z * Q[2];
	qy.z  = mx->z * Q[4] + my->z * Q[5] + mz->z * Q[6];
	qz.z  = mx->z * Q[8] + my->z * Q[9] + mz->z * Q[10];

	//t = q * Q^(-1) = q * Q^t
	tx->x = qx.x * Q[0] + qx.y * Q[1] + qx.z * Q[2];
	ty->x = qy.x * Q[0] + qy.y * Q[1] + qy.z * Q[2];
	tz->x = qz.x * Q[0] + qz.y * Q[1] + qz.z * Q[2];

	tx->y = qx.x * Q[4] + qx.y * Q[5] + qx.z * Q[6];
	ty->y = qy.x * Q[4] + qy.y * Q[5] + qy.z * Q[6];
	tz->y = qz.x * Q[4] + qz.y * Q[5] + qz.z * Q[6];

	tx->z = qx.x * Q[8] + qx.y * Q[9] + qx.z * Q[10];
	ty->z = qy.x * Q[8] + qy.y * Q[9] + qy.z * Q[10];
	tz->z = qz.x * Q[8] + qz.y * Q[9] + qz.z * Q[10];

	//Set coordinate system
	tx->coordinate_system = q->coordinate_system;
	tx->derivative_level = mx->derivative_level;
	ty->coordinate_system = q->coordinate_system;
	ty->derivative_level = my->derivative_level;
	tz->coordinate_system = q->coordinate_system;
	tz->derivative_level = mz->derivative_level;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Invert symmetric 3x3 tensor built out of vectors
///
/// If the tensor has the following form (inertia tensor form):
/// \f[
///		m =
///		\left[ \begin{array}{cccc} 
///			m_{xx} & m_{xy} & m_{xz} \\ 
///			m_{xy} & m_{yy} & m_{yz} \\ 
///			m_{xz} & m_{yz} & m_{zz}
///		\end{array} \right]
/// \f]
///
/// The inverted tensor is:
/// \f[
///		m^{-1} =
///		\dfrac{1}{\Delta}
///		\left[ \begin{array}{cccc} 
///			k_1 & k_2 & k_3 \\ 
///			k_2 & k_4 & k_5 \\ 
///			k_3 & k_5 & k_6
///		\end{array} \right]
/// \f]
///
/// where
/// \f{eqnarray*}{
///		\Delta &=& m_{xx} m_{yy} m_{zz} - 2 m_{xy} m_{yz} m_{zx} - m_{xx} m_{yz}^2 -
///			m_{yy} m_{zx}^2 - m_{zz} m_{xy}^2 \\
///		k_1 &=& \dfrac{1}{\Delta} (m_{yy} m_{zz} - m_{yz}^2) \\
///		k_2 &=& \dfrac{1}{\Delta} (m_{yz} m_{zx} + m_{xy} m_{zz}) \\
///		k_3 &=& \dfrac{1}{\Delta} (m_{xy} m_{yz} + m_{zx} m_{yy}) \\
///		k_4 &=& \dfrac{1}{\Delta} (m_{zz} m_{xx} - m_{zx}^2) \\
///		k_5 &=& \dfrac{1}{\Delta} (m_{xy} m_{zx} + m_{yz} m_{xx}) \\
///		k_6 &=& \dfrac{1}{\Delta} (m_{xx} m_{yy} - m_{xy}^2) \\
/// \f}
///
/// @bug Assert is triggered in evds_body.c when tensor is accumulated for many bodies. For some reason,
///   error in zy/yz components slightly exceeds EVDS_EPS.
///
/// @returns Inverted tensor
////////////////////////////////////////////////////////////////////////////////
void EVDS_Tensor_InvertSymmetric(EVDS_VECTOR* tx, EVDS_VECTOR* ty, EVDS_VECTOR* tz,
								 EVDS_VECTOR* mx, EVDS_VECTOR* my, EVDS_VECTOR* mz) {
	EVDS_REAL k1,k2,k3,k4,k5,k6;
	EVDS_REAL D1;

	//Check symmetric form of the tensor
	//FIXME: possible bug in tensor accumulation, Mzy-Myz > EVDS_EPS
	//EVDS_ASSERT(fabs(my->x - mx->y) < EVDS_EPS);
	//EVDS_ASSERT(fabs(mz->x - mx->z) < EVDS_EPS);
	//EVDS_ASSERT(fabs(mz->y - my->z) < EVDS_EPS);

	//Compute determinant
	D1 = 1.0 / (mx->x*my->y*mz->z - 2*mx->y*my->z*mz->x - 
				mx->x*my->z*my->z - my->y*mz->x*mz->x - mz->z*mx->y*mx->y);

	//Invert tensor
	k1 = (my->y*mz->z - my->z*my->z) * D1;
	k2 = (my->z*mz->x + mx->y*mz->z) * D1;
	k3 = (mx->y*my->z + mz->x*my->y) * D1;
	k4 = (mz->z*mx->x - mz->x*mz->x) * D1;
	k5 = (mx->y*mz->x + my->z*mx->x) * D1;
	k6 = (mx->x*my->y - mx->y*mx->y) * D1;

	tx->x = k1; tx->y = k2; tx->z = k3;
	ty->x = k2; ty->y = k4; ty->z = k5;
	tz->x = k3; tz->y = k5; tz->z = k6;

	//Set coordinate system
	tx->coordinate_system = mx->coordinate_system;
	ty->coordinate_system = my->coordinate_system;
	tz->coordinate_system = mz->coordinate_system;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Invert 3x3 tensor build out of vectors
///
/// For a tensor of general form:
/// \f[
///		m =
///		\left[ \begin{array}{cccc} 
///			m_{xx} & m_{xy} & m_{xz} \\ 
///			m_{yx} & m_{yy} & m_{yz} \\ 
///			m_{zx} & m_{zy} & m_{zz}
///		\end{array} \right]
/// \f]
///
/// The inverted tensor is:
/// \f[
///		m^{-1} =
///		\dfrac{1}{\Delta}
///		\left[ \begin{array}{cccc} 
///			k_1 & k_2 & k_3 \\ 
///			k_4 & k_5 & k_6 \\ 
///			k_7 & k_8 & k_9
///		\end{array} \right]
/// \f]
///
/// where
/// \f{eqnarray*}{
///		\Delta &=& m_{xx} (m_{yy} m_{zz} - m_{yz} m_{zy}) +
///				   m_{xy} (m_{yz} m_{zx} - m_{yx} m_{zz}) +
///				   m_{xz} (m_{yx} m_{zy} - m_{yy} m_{zx}) \\
///		k_1 &=& \dfrac{1}{\Delta} (-m_{yz} m_{zy} + m_{yy} m_{zz}) \\
///		k_2 &=& \dfrac{1}{\Delta} ( m_{xz} m_{zy} - m_{xy} m_{zz}) \\
///		k_3 &=& \dfrac{1}{\Delta} (-m_{xz} m_{yy} + m_{xy} m_{yz}) \\
///		k_4 &=& \dfrac{1}{\Delta} ( m_{yz} m_{zx} - m_{yx} m_{zz}) \\
///		k_5 &=& \dfrac{1}{\Delta} (-m_{xz} m_{zx} + m_{xx} m_{zz}) \\
///		k_6 &=& \dfrac{1}{\Delta} ( m_{xz} m_{yx} - m_{xx} m_{yz}) \\
///		k_7 &=& \dfrac{1}{\Delta} (-m_{yy} m_{zx} + m_{yx} m_{zy}) \\
///		k_8 &=& \dfrac{1}{\Delta} ( m_{xy} m_{zx} - m_{xx} m_{zy}) \\
///		k_9 &=& \dfrac{1}{\Delta} (-m_{xy} m_{yx} + m_{xx} m_{yy}) \\
/// \f}
///
/// @returns Inverted tensor
////////////////////////////////////////////////////////////////////////////////
void EVDS_Tensor_Invert(EVDS_VECTOR* tx, EVDS_VECTOR* ty, EVDS_VECTOR* tz,
						EVDS_VECTOR* mx, EVDS_VECTOR* my, EVDS_VECTOR* mz) {

	EVDS_REAL k1,k2,k3,k4,k5,k6,k7,k8,k9;
	EVDS_REAL D1;

	//Compute determinant
	D1 = 1.0 / (mx->x*(my->y*mz->z - my->z*mz->y)+
				mx->y*(my->z*mz->x - my->x*mz->z)+
				mx->z*(my->x*mz->y - my->y*mz->x));

	//Invert tensor
	k1 = (-my->z * mz->y + my->y * mz->z) * D1;
	k2 = ( mx->z * mz->y - mx->y * mz->z) * D1;
	k3 = (-mx->z * my->y + mx->y * my->z) * D1;
	k4 = ( my->z * mz->x - my->x * mz->z) * D1;
	k5 = (-mx->z * mz->x + mx->x * mz->z) * D1;
	k6 = ( mx->z * my->x - mx->x * my->z) * D1;
	k7 = (-my->y * mz->x + my->x * mz->y) * D1;
	k8 = ( mx->y * mz->x - mx->x * mz->y) * D1;
	k9 = (-mx->y * my->x + mx->x * my->y) * D1;

	tx->x = k1; tx->y = k2; tx->z = k3;
	ty->x = k4; ty->y = k5; ty->z = k6;
	tz->x = k7; tz->y = k8; tz->z = k9;

	//Set coordinate system
	tx->coordinate_system = mx->coordinate_system;
	ty->coordinate_system = my->coordinate_system;
	tz->coordinate_system = mz->coordinate_system;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Transposes the matrix
////////////////////////////////////////////////////////////////////////////////
void EVDS_Matrix_Transpose(EVDS_MATRIX target, EVDS_MATRIX source) {
	int i,j;
	EVDS_MATRIX result;
	for (i = 0; i < 4; i++) {
		for (j = 0; j < 4; j++) {
			result[i*4+j] = source[j*4+i];
		}
	}

	memcpy(target,result,sizeof(EVDS_MATRIX));
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Check whether tensor is a symmetric 3x3 tensor.
///
/// Checks whether the tensor has the following form (inertia tensor form):
/// \f[
///		m =
///		\left[ \begin{array}{cccc} 
///			m_{xx} & m_{xy} & m_{xz} \\ 
///			m_{xy} & m_{yy} & m_{yz} \\ 
///			m_{xz} & m_{yz} & m_{zz}
///		\end{array} \right]
/// \f]
///
/// @returns Inverted tensor
////////////////////////////////////////////////////////////////////////////////
void EVDS_Tensor_IsSymmetric(EVDS_VECTOR* mx, EVDS_VECTOR* my, EVDS_VECTOR* mz, int* is_symmetric) {
	*is_symmetric = 1;
	*is_symmetric &= (fabs(my->x - mx->y) < EVDS_EPS);
	*is_symmetric &= (fabs(mz->x - mx->z) < EVDS_EPS);
	*is_symmetric &= (fabs(mz->y - my->z) < EVDS_EPS);
}




////////////////////////////////////////////////////////////////////////////////
/// @brief Initializes state vector to an empty state vector in target coordinates
////////////////////////////////////////////////////////////////////////////////
void EVDS_StateVector_Initialize(EVDS_STATE_VECTOR* v, EVDS_OBJECT* target_coordinates) {
	memset(v,0,sizeof(EVDS_STATE_VECTOR));
	v->time = SIMC_Thread_GetMJDTime();
	v->position.coordinate_system = target_coordinates;
	v->position.derivative_level = EVDS_VECTOR_POSITION;
	v->velocity.coordinate_system = target_coordinates;
	v->velocity.derivative_level = EVDS_VECTOR_VELOCITY;
	v->acceleration.coordinate_system = target_coordinates;
	v->acceleration.derivative_level = EVDS_VECTOR_ACCELERATION;
	v->orientation.coordinate_system = target_coordinates;
	v->angular_velocity.coordinate_system = target_coordinates;
	v->angular_velocity.derivative_level = EVDS_VECTOR_ANGULAR_VELOCITY;
	v->angular_acceleration.coordinate_system = target_coordinates;
	v->angular_acceleration.derivative_level = EVDS_VECTOR_ANGULAR_ACCELERATION;
	v->orientation.q[0] = 1.0;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Initializes state vector derivative to an empty state vector derivative in target coordinates
////////////////////////////////////////////////////////////////////////////////
void EVDS_StateVector_Derivative_Initialize(EVDS_STATE_VECTOR_DERIVATIVE* v, EVDS_OBJECT* target_coordinates) {
	memset(v,0,sizeof(EVDS_STATE_VECTOR_DERIVATIVE));
	v->velocity.coordinate_system = target_coordinates;
	v->velocity.derivative_level = EVDS_VECTOR_VELOCITY;
	v->acceleration.coordinate_system = target_coordinates;
	v->acceleration.derivative_level = EVDS_VECTOR_ACCELERATION;
	v->angular_velocity.coordinate_system = target_coordinates;
	v->angular_velocity.derivative_level = EVDS_VECTOR_ANGULAR_VELOCITY;
	v->angular_acceleration.coordinate_system = target_coordinates;
	v->angular_acceleration.derivative_level = EVDS_VECTOR_ANGULAR_ACCELERATION;

	v->force.coordinate_system = target_coordinates;
	v->force.derivative_level = EVDS_VECTOR_FORCE;
	v->torque.coordinate_system = target_coordinates;
	v->torque.derivative_level = EVDS_VECTOR_TORQUE;
}




////////////////////////////////////////////////////////////////////////////////
/// @brief Add vector v1 to vector v2
////////////////////////////////////////////////////////////////////////////////
void EVDS_ShortVector_Add(EVDS_SHORT_VECTOR* target, EVDS_SHORT_VECTOR* v1, EVDS_SHORT_VECTOR* v2) {
	target->x = v1->x + v2->x;
	target->y = v1->y + v2->y;
	target->z = v1->z + v2->z;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Subtract vector v2 from vector v1
////////////////////////////////////////////////////////////////////////////////
void EVDS_ShortVector_Subtract(EVDS_SHORT_VECTOR* target, EVDS_SHORT_VECTOR* v1, EVDS_SHORT_VECTOR* v2) {
	target->x = v1->x - v2->x;
	target->y = v1->y - v2->y;
	target->z = v1->z - v2->z;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Finds cross-product between vectors v1 and v2
////////////////////////////////////////////////////////////////////////////////
void EVDS_ShortVector_Cross(EVDS_SHORT_VECTOR* target, EVDS_SHORT_VECTOR* v1, EVDS_SHORT_VECTOR* v2) {
	EVDS_REAL x, y, z;

	x = v1->y*v2->z - v1->z*v2->y;
	y = v1->z*v2->x - v1->x*v2->z;
	z = v1->x*v2->y - v1->y*v2->x;
	target->x = x;
	target->y = y;
	target->z = z;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Find dot product between vector v1 and v2 in coordinate system of vector v1
////////////////////////////////////////////////////////////////////////////////
void EVDS_ShortVector_Dot(EVDS_REAL* target, EVDS_SHORT_VECTOR* v1, EVDS_SHORT_VECTOR* v2) {
	*target = v1->x*v2->x + v1->y*v2->y + v1->z*v2->z;
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Returns a normalized direction vector of v
////////////////////////////////////////////////////////////////////////////////
void EVDS_ShortVector_Normalize(EVDS_SHORT_VECTOR* target, EVDS_SHORT_VECTOR* v) {
	EVDS_REAL mag = sqrt(v->x*v->x + v->y*v->y + v->z*v->z);
	if (mag == 0.0) {
		target->x = 0.0;
		target->y = 0.0;
		target->z = 0.0;
	}
	else {
		target->x = v->x / mag;
		target->y = v->y / mag;
		target->z = v->z / mag;
	}
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Return vector length
////////////////////////////////////////////////////////////////////////////////
void EVDS_ShortVector_Length(EVDS_REAL* target, EVDS_SHORT_VECTOR* v) {
	*target = sqrt(v->x*v->x + v->y*v->y + v->z*v->z);
}


////////////////////////////////////////////////////////////////////////////////
/// @brief
////////////////////////////////////////////////////////////////////////////////
void EVDS_ShortVector_Distance(EVDS_REAL* target, EVDS_SHORT_VECTOR* v1, EVDS_SHORT_VECTOR* v2)
{
	*target = sqrt(
		(v1->x - v2->x)*(v1->x - v2->x) +
		(v1->y - v2->y)*(v1->y - v2->y) +
		(v1->z - v2->z)*(v1->z - v2->z));
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Check if two vectors are equal to eachother
////////////////////////////////////////////////////////////////////////////////
int EVDS_ShortVector_Equal(EVDS_SHORT_VECTOR* v1, EVDS_SHORT_VECTOR* v2) {
	EVDS_REAL distance2 =
		(v1->x - v2->x)*(v1->x - v2->x) +
		(v1->y - v2->y)*(v1->y - v2->y) +
		(v1->z - v2->z)*(v1->z - v2->z);
	return distance2 < EVDS_EPSf;
}
