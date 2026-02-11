#pragma once

#include <cmath>

#include "Vector3D.h"
#include "../data/Constants.h"

using namespace std;

// Value that combines information about fuel air travel to simplify equations
static double mu = (AIR_DENSITY * DRAG_COEF * AREA) / (2 * MASS);

// Struct that contains information about a projecties position and velocity
// Contains function to compute the instantaneous acceleration of the projectile
struct State {
	Vector3D position, velocity;

	Vector3D getAcceleration(bool usingDrag) {
		Vector3D accel = {
			-(mu * usingDrag)	* velocity.x * velocity.mag(),
			-(mu * usingDrag) * velocity.y * velocity.mag(),
			-GRAVITY -(mu * usingDrag) * velocity.z * velocity.mag()
		};

		return accel;
	}
};

