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
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "evds.h"


////////////////////////////////////////////////////////////////////////////////
/// @brief Create datum based on the object.
///
/// This function returns datum derived from object parameters. If no object is
/// specified or the object is not a planetary body, a datum corresponding to
/// sphere with zero-radius is returned.
///
/// If planetary body has `geometry.semimajor_axis` variable defined, the datum will be
/// derieved from planets ellipsoid based on one of the following parameters
/// (\f$a\f$ is semimajor axis, \f$b\f$ is semiminor axis):
/// Variable						| Formula used
/// --------------------------------|----------------------
/// `geometry.semiminor_axis`		| Datum derived from semimajor and semiminor axes directly
/// `geometry.flattening`			| \f$b = a (1 - flattening)\f$
/// `geometry.inverse_flattening`	| \f$b = a (1 - \frac{1}{inverse\_ flattening})\f$
///
/// If planetary body has only `geometry.radius` defined, the datum will be derived for a
/// spherical body with given radius.
///
/// @returns Datum from given object
/// @param[out] datum Datum will be saved here
/// @param[in] object Object, from which datum will be derived
////////////////////////////////////////////////////////////////////////////////
void EVDS_Geodetic_DatumFromObject(EVDS_GEODETIC_DATUM* datum, EVDS_OBJECT* object) {
	//Default: datum corresponding to bearing/elevation around vessels reference point
	datum->semimajor_axis = 0;
	datum->semiminor_axis = 0;
	datum->object = object;

	//If no object passed, don't try to guess anything
	if (!object) return;

	//Determine datum based on celestial body parameters
	if (EVDS_Object_CheckType(object,"planet") == EVDS_OK) {
		EVDS_VARIABLE* variable;

		//Planet is an ellipsoid
		if (EVDS_Object_GetVariable(object,"geometry.semimajor_axis",&variable) == EVDS_OK) {
			//Presence of semi-major axis defines an ellipsoid
			EVDS_Variable_GetReal(variable,&datum->semimajor_axis);

			if (EVDS_Object_GetVariable(object,"geometry.semiminor_axis",&variable) == EVDS_OK) {
				//Semi-minor axis defined
				EVDS_Variable_GetReal(variable,&datum->semiminor_axis);
			} else if (EVDS_Object_GetVariable(object,"geometry.flattening",&variable) == EVDS_OK) {
				//Flattening is defined
				EVDS_REAL f;
				EVDS_Variable_GetReal(variable,&f);

				datum->semiminor_axis = datum->semimajor_axis * (1 - f);
			} else if (EVDS_Object_GetVariable(object,"geometry.inverse_flattening",&variable) == EVDS_OK) {
				//Inverse of flattening is defined
				EVDS_REAL inv_f;
				EVDS_Variable_GetReal(variable,&inv_f);

				if (inv_f != 0.0) {
					datum->semiminor_axis = datum->semimajor_axis * (1 - 1/inv_f);
				} else {
					datum->semiminor_axis = 0.0;
				}
			} else {
				//No information about ellipsoid shape
				datum->semiminor_axis = datum->semimajor_axis;
			}
		} else if (EVDS_Object_GetVariable(object,"geometry.radius",&variable) == EVDS_OK) {
			//Only radius is defined, use spheric datum
			EVDS_Variable_GetReal(variable,&datum->semimajor_axis);
			datum->semiminor_axis = datum->semimajor_axis;
		}
	}
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Set geodetic coordinate around a given body.
///
/// Datum will be automatically derived from the object around which coordinates are specified.
/// If longitude value greater or equal to 180.0 is passed, it will be remapped into \f$[-180.0, 180.0)\f$ range.
///
/// @returns Geodetic coordinate with defined datum
/// @param[out] coordinate Geodetic coordinate
/// @param[in] object Object, around which coordinates are specified
/// @param[in] latitude Geodetic latitude
/// @param[in] longitude Geodetic longitude
/// @param[in] elevation Geodetic elevation (height)
////////////////////////////////////////////////////////////////////////////////
void EVDS_Geodetic_Set(EVDS_GEODETIC_COORDINATE* coordinate, EVDS_OBJECT* object,
					   EVDS_REAL latitude, EVDS_REAL longitude, EVDS_REAL elevation) {
	if (longitude >= 180.0) {
		longitude = -180.0 + fmod(longitude - 180.0,360.0);
	}

	coordinate->latitude = latitude;
	coordinate->longitude = longitude;
	coordinate->elevation = elevation;
	EVDS_Geodetic_DatumFromObject(&coordinate->datum,object);
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Converts geodetic coordinates around object to a position vector.
///
/// First the eccentricity and normal of an ellipsoid are calculated. \f$a\f$ is the semimajor axis,
/// \f$b\f$ is the semiminor axis.
/// \f{eqnarray*}{
///		e^2 &=& 1 - \frac{b^2}{a^2} \\
///		R_N &=& \frac{a}{\sqrt{1 - e^2 sin^2(latitude)}}
/// \f}
///
/// The cartesian coordinates are calculated as:
/// \f{eqnarray*}{
///		x &=& (R_N + elevation) \ cos(longitude) \ cos(latitude) \\
///		y &=& (R_N + elevation) \ sin(longitude) \ cos(latitude) \\
///		z &=& (R_N (1 - e^2) + elevation) \ sin(latitude)
/// \f}
///
/// @note The geodetic coordinate must have proper datum defined. See EVDS_Geodetic_Set().
///
/// @returns Position vector based on geodetic coordinate.
/// @param[out] target Position vector in coordinate system of body, around which geodetic coordinates are specified.
/// @param[in] source Geodetic coordinate with correctly defined datum.
////////////////////////////////////////////////////////////////////////////////
void EVDS_Geodetic_ToVector(EVDS_VECTOR* target, EVDS_GEODETIC_COORDINATE* source) {
	EVDS_REAL x,y,z; //Components of the result
	EVDS_REAL eccentricity_squared,normal; //Ellipsoid parameters
	EVDS_REAL sin_lat,cos_lat,sin_lon,cos_lon; //Sines/cosines
	if (!target) return;
	if (!source) return;

	//Perform some self-checks
	EVDS_ASSERT(source->datum.object);
	EVDS_ASSERT(source->datum.semiminor_axis <= source->datum.semimajor_axis);

	//Calculate sines and cosines
	sin_lat = sin(EVDS_RAD(source->latitude));
	cos_lat = cos(EVDS_RAD(source->latitude));
	sin_lon = sin(EVDS_RAD(source->longitude));
	cos_lon = cos(EVDS_RAD(source->longitude));
	
	//Calculate eccentricity of ellipsoid
	// The 'if' excludes invalid inputs and optimizes calculation for spheres
	if (source->datum.semiminor_axis < source->datum.semimajor_axis) {
		eccentricity_squared = 1 - 
			(source->datum.semiminor_axis*source->datum.semiminor_axis)/
			(source->datum.semimajor_axis*source->datum.semimajor_axis);
	} else {
		eccentricity_squared = 0.0;
	}

	//Calculate normal
	if (eccentricity_squared > 0.0) {
		normal = source->datum.semimajor_axis / sqrt(1 - eccentricity_squared * sin_lat * sin_lat);
	} else {
		normal = source->datum.semimajor_axis;
	}

	//Convert geographic coordinates to X,Y,Z
	x = (normal + source->elevation)*cos_lon*cos_lat;
	y = (normal + source->elevation)*sin_lon*cos_lat;
	z = (normal*(1 - eccentricity_squared) + source->elevation)*sin_lat;

	//Set vector in target coordinates
	EVDS_Vector_Set(target,EVDS_VECTOR_POSITION,source->datum.object,x,y,z);
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Converts position vector into geodetic coordinate.
///
/// The target coordinate datum is determined from `target_datum` parameter or
/// automatically derived from coordinate system of `source` vector.
///
/// The equations for conversion are listed below. \f$a\f$ is the semimajor axis,
/// \f$b\f$ is the semiminor axis.
/// If the target datum is spherical, a simple spherical coordinates conversion is used:
/// \f{eqnarray*}{
///		longitude &=& arctg(\frac{y}{x}) = arctg2(y,x) \\
///		latitude &=& arcsin(\frac{z}{r}) \\
///		elevation &=& \sqrt{x^2 + y^2 + z^2} - a
/// \f}
///
/// If datum is defined with a reference ellipsoid, newtons iteration method must be used to
/// solve Bowring's irrational geodetic-latitude equation:
/// \f{eqnarray*}{
///		\kappa - 1 - \frac{e^2 a \kappa}{\sqrt{p^2 + (1 - e^2)z^2 \kappa^2}} = 0
/// \f}
/// where:
/// \f{eqnarray*}{
///		\kappa &=& \frac{p}{z} tan(latitude) \\
///		\kappa_0 &=& (1 - e^2)^{-1} \\
///		p &=& \sqrt{x^2 + y^2} \\
///		e^2 &=& 1 - \frac{b^2}{a^2}
/// \f}
///
/// The following iteration is used until \f$\kappa\f$ converges with EVDS_EPSf precision,
/// but no more than 8 iterations (but typically converges in 2-3 iterations):
/// \f{eqnarray*}{
///		c_i &=& \frac{(p^2 + (1 - e^2) z^2 \kappa_i^2)^{3/2}}{a e^2} \\
///		\kappa_{i+1} &=& 1 + \frac{p^2 + (1 - e^2) z^2 \kappa_i^3}{c_i - p^2}
/// \f}
///
/// So the final expression for latitude becomes:
/// \f{eqnarray*}{
///		tan(latitude) &=&
///			\begin{cases}
///			\frac{\kappa z}{p}, & p \neq 0 \\
///			\infty, & p = 0, \ \kappa z \gt 0 \\
///			-\infty, & p = 0, \ \kappa z \le 0 \\
///			\end{cases} \\
///		latitude &=& atan(tan(latitude))
/// \f}
///
/// The elevation can be determined from ellipsoid geometry, and longitude determined as in spheric datum case:
/// \f{eqnarray*}{
///		R_N &=& \frac{a}{\sqrt{1 - e^2 sin^2(latitude)}} \\
///		elevation &=& 
///			\begin{cases}
///			\frac{z}{sin(latitude)} - R_N (1 - e^2), & sin(latitude) \neq 0 \\
///			p - R_N, & sin(latitude) = 0 \\
///			\end{cases} \\
///		longitude &=& arctg(\frac{y}{x}) = arctg2(y,x)
/// \f}
///
/// @note The function returns longitude in range of \f$[-180.0, 180.0)\f$, excluding longitude 180.0.
///       If longitude calculations result in 180.0 degrees, it will be returned as -180.0 degrees.
///
/// @returns Geodetic position based on position vector.
/// @param[out] target Geodetic coordinate in target datum.
/// @param[in] source Position vector.
/// @param[in] target_datum Datum that must be used for the target coordinate.
////////////////////////////////////////////////////////////////////////////////
void EVDS_Geodetic_FromVector(EVDS_GEODETIC_COORDINATE* target, EVDS_VECTOR* source, EVDS_GEODETIC_DATUM* target_datum) {
	EVDS_REAL x,y,z; //Components of the result
	if (!target) return;
	if (!source) return;

	//Input checks
	EVDS_ASSERT(source->coordinate_system);
	EVDS_ASSERT(source->derivative_level == EVDS_VECTOR_POSITION);

	//Determine datum based on vector coordinate system object if needed
	if (!target_datum) {
		EVDS_Geodetic_DatumFromObject(&target->datum,source->coordinate_system);
	} else {
		memcpy(&target->datum,target_datum,sizeof(EVDS_GEODETIC_DATUM));
	}

	//Get vector components relative to datum body
	EVDS_Vector_Get(source,&x,&y,&z,target->datum.object);

	//Check if datum is spheric
	if (target->datum.semimajor_axis == target->datum.semiminor_axis) {
		EVDS_REAL r = sqrt(x*x+y*y+z*z)+EVDS_EPS;

		target->longitude	= EVDS_DEG(atan2(y,x));
		target->latitude	= EVDS_DEG(asin(z/r));
		target->elevation	= r - target->datum.semimajor_axis;
		if (target->longitude == 180.0) target->longitude = -180.0;
	} else { //Non-spheric datum
		EVDS_REAL eccentricity_squared,normal;
		EVDS_REAL p,k,k_prev,k0,ci;
		EVDS_REAL lat,tan_lat,sin_lat;
		int iterations;

		//Calculate some information about ellipsoid
		eccentricity_squared = 1 - 
			(target->datum.semiminor_axis*target->datum.semiminor_axis)/
			(target->datum.semimajor_axis*target->datum.semimajor_axis);
		k0 = 1/(1 - eccentricity_squared);
		p = sqrt(x*x+y*y);

		//Calculate longitude
		target->longitude = EVDS_DEG(atan2(y,x));
		if (target->longitude == 180.0) target->longitude = -180.0;

		//Use newtons method to calculate latitude
		k = k0; //k0 as initial guess gives convergion within 2-3 iterations
		k_prev = EVDS_INFINITY; //No good previous value
		iterations = 0;

		while ((fabs(k-k_prev) > EVDS_EPS) && (iterations < 8)) {
			k_prev = k;

			ci = pow(p*p + (1 - eccentricity_squared)*z*z*k*k,3.0/2.0)/
				(target->datum.semimajor_axis*eccentricity_squared);
			k = 1 + (p*p + (1 - eccentricity_squared)*z*z*k*k*k) / (ci - p*p);
			iterations++;
		}

		//Determine tangent of latitude from k
		if (p != 0.0) {
			tan_lat = k*z/p;
		} else { //One of the poles
			if (k*z > 0) { //North pole
				tan_lat = EVDS_INFINITY;
			} else { //South pole
				tan_lat = -EVDS_INFINITY;
			}
		}
		lat = atan(tan_lat);
		target->latitude = EVDS_DEG(lat);

		//Calculate elevation around ellipsoid
		sin_lat = sin(lat);
		normal = target->datum.semimajor_axis / sqrt(1 - eccentricity_squared * sin_lat * sin_lat);
		if (sin_lat != 0.0) {
			target->elevation = z/sin_lat - normal*(1 - eccentricity_squared);
		} else { //Special case when located on equator
			target->elevation = p - normal;
		}
	}
}




////////////////////////////////////////////////////////////////////////////////
/// @brief Get state vector of the LVLH frame.
///
/// Returns state vector that can be used for an object to represent LVLH coordinate frame.
/// The state vector is returned in the coordinates of the object referenced by
/// EVDS_GEODETIC_COORDINATE datum.
///
/// @note The coordinate frame will only be created as non-inertial if the geodetic coordinate
///       datum references a rotating planet/rotating coordinate frame!
///
/// Example of use:
/// ~~~{.c}
///		EVDS_STATE_VECTOR vector;
///		EVDS_GEODETIC_COORDINATE geodetic_coordinate;
///
///		//Create LVLH coordinate frame
///		EVDS_Object_Create(earth,&frame);
///
///		//Place LVLH coordinate frame at some point on the planet
///		EVDS_Geodetic_Set(&geodetic_coordinate,earth,50.45,30.52,0);
///		EVDS_LVLH_GetStateVector(&vector,&geodetic_coordinate);
///		EVDS_Object_SetStateVector(frame,&vector);
/// ~~~
///
/// @returns State vector for a LVLH coordinate frame.
/// @param[out] target State vector corresponding to LVLH coordinate frame.
/// @param[in] coordinate Geodetic coordinate of the LVLH frame origin.
////////////////////////////////////////////////////////////////////////////////
void EVDS_LVLH_GetStateVector(EVDS_STATE_VECTOR* target, EVDS_GEODETIC_COORDINATE* coordinate) {
	//Initialize state vector
	EVDS_StateVector_Initialize(target, coordinate->datum.object);

	//Rotate and translate coordinate frame
	EVDS_LVLH_QuaternionFromLVLH(&target->orientation, &target->orientation, coordinate);
	EVDS_Geodetic_ToVector(&target->position, coordinate);
}




////////////////////////////////////////////////////////////////////////////////
/// @brief Convert quaternion to objects LVLH frame.
///
/// Returns quaternion in coordinate frame of the object referenced by EVDS_GEODETIC_COORDINATE
/// datum, rotated to represent the attitude in LVLH coordinate frame.
///
/// @note Although the quaternion represents attitude in LVLH coordinate system, it will be returned (numerically)
///       in coordinate system of the object referenced by datum (e.g. planet-surface relative coordinates).
///
/// The numerical values of the returned quaternion will correspond to attitude in the LVLH coordinate frame.
/// Here's an example of converting aircraft attitude into LVLH euler angles (horizon-relative pitch, yaw, roll):
/// ~~~{.c}
///		EVDS_OBJECT* aircraft;
///		EVDS_STATE_VECTOR vector;
///		EVDS_QUATERNION attitude; //LVLH attitude
///		EVDS_GEODETIC_COORDINATE position; //Geodetic position
///		EVDS_REAL pitch,yaw,roll; //Horizon-relative Euler angles
///
///		EVDS_Object_GetStateVector(aircraft,&vector);
///		EVDS_Geodetic_FromVector(&position,&vector.position,0); //Automatic datum
///		EVDS_LVLH_QuaternionToLVLH(&attitude,&vector.orientation,position); //Determine LVLH attitude
///
///		//Convert quaternion into angles numerically (without involving conversion)
///		EVDS_Quaternion_ToEuler(&attitude,attitude.coordinate_system,&roll,&pitch,&yaw);
///		roll = EVDS_DEG(roll);
///		pitch = EVDS_DEG(pitch);
///		yaw = EVDS_DEG(yaw);
/// ~~~
///
/// @returns Quaternion in LVLH frame, in coordinates of object referenced by datum.
/// @param[out] target_lvlh Quaternion in LVLH coordinate frame.
/// @param[in] source Source quaternion.
/// @param[in] coordinate Geodetic coordinate of the LVLH frame origin.
////////////////////////////////////////////////////////////////////////////////
void EVDS_LVLH_QuaternionToLVLH(EVDS_QUATERNION* target_lvlh, EVDS_QUATERNION* source, EVDS_GEODETIC_COORDINATE* coordinate) {
	EVDS_QUATERNION q_lon,q_lat; //Rotations to go from North pole to given latitude/longitude
	EVDS_Quaternion_FromEuler(&q_lon,coordinate->datum.object,0,0,EVDS_RAD(coordinate->longitude));
	EVDS_Quaternion_FromEuler(&q_lat,coordinate->datum.object,0,EVDS_RAD(90-coordinate->latitude),0);

	//Rotate quaternion Q from inertial to LVLH coordinates
	EVDS_Quaternion_Convert(target_lvlh,source,coordinate->datum.object);
	EVDS_Quaternion_MultiplyConjugatedQ(target_lvlh,&q_lon,target_lvlh);
	EVDS_Quaternion_MultiplyConjugatedQ(target_lvlh,&q_lat,target_lvlh);
}


////////////////////////////////////////////////////////////////////////////////
/// @brief Convert quaternion from objects LVLH frame.
///
/// Returns quaternion in coordinate frame of the object referenced by EVDS_GEODETIC_COORDINATE
/// datum, rotated from LVLH coordinate frame.
///
/// @note The source quaternion in LVLH coordinate frame must be specified in coordinate frame of the object
///       referenced by the geodetic coordinates datum.
///
/// Here's an example of initializing aircraft altitude based on horizon-relative Euler angles:
/// ~~~{.c}
///		EVDS_OBJECT* aircraft;
///		EVDS_STATE_VECTOR vector;
///		EVDS_QUATERNION attitude; //LVLH attitude
///		EVDS_GEODETIC_COORDINATE position; //Geodetic position
///
///		EVDS_Object_GetStateVector(aircraft,&vector);
///		EVDS_Geodetic_FromVector(&position,&vector.position,0); //Automatic datum
///
///		//Create quaternion numerically equivalent to required attitude, in planets coordinate system
///		EVDS_Quaternion_FromEuler(&attitude,position.datum.object,EVDS_DEG(roll),EVDS_DEG(pitch),EVDS_DEG(yaw));
///		EVDS_LVLH_QuaternionFromLVLH(&attitude,&attitude,position);
///		EVDS_Object_SetOrientationQuaternion(aircraft,&attitude);
/// ~~~
///
/// @returns Quaternion in LVLH frame, in coordinates of object referenced by datum.
/// @param[out] target Target quaternion.
/// @param[in] source_lvlh Quaternion in LVLH coordinate frame.
/// @param[in] coordinate Geodetic coordinate of the LVLH frame origin.
////////////////////////////////////////////////////////////////////////////////
void EVDS_LVLH_QuaternionFromLVLH(EVDS_QUATERNION* target, EVDS_QUATERNION* source_lvlh, EVDS_GEODETIC_COORDINATE* coordinate) {
	EVDS_QUATERNION q_lon,q_lat; //Rotations to go from North pole to given latitude/longitude
	EVDS_Quaternion_FromEuler(&q_lon,coordinate->datum.object,0,0,EVDS_RAD(coordinate->longitude));
	EVDS_Quaternion_FromEuler(&q_lat,coordinate->datum.object,0,EVDS_RAD(90-coordinate->latitude),0);

	//Rotate quaternion Q from LVLH to inertial coordinates
	EVDS_Quaternion_Convert(target,source_lvlh,coordinate->datum.object);
	EVDS_Quaternion_Multiply(target,&q_lat,target);
	EVDS_Quaternion_Multiply(target,&q_lon,target);
}
