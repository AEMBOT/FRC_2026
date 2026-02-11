#pragma once

#include <cmath>
#include <iostream>

#include "../structure/Vector3D.h"
#include "../structure/State.h"

using namespace std;

class ProjectileSim {
	private:
		// Position and velocity of the simulated projectile
		State projectileState;

		// Whether or not to simulate drag
		bool usesDrag;
	
	public:
		/**
		 * Class for simulating projectile motion which takes into account air resistance
		 *
		 * @param initPos initial position of the projectile
		 * @param initVel initial velocity of the projectile
		 * @param useDrag whether or not to simulate drag
		 */
		ProjectileSim(Vector3D initPos, Vector3D initVel, bool useDrag);
		/**
		 * Artifically set the position and/or velocity of the projectile 
		 *
		 * @param inState new projectile state
		 */
		void SetProjectileState(State inState);
		/**
		 * @return the current projectile position and velocity
		 */
		State GetProjectileState();
		/**
		 * Run one step of the projectile simulation
		 *
		 * More specifically, integrate the projectile state with RK4
		 * Then update position and velocity based on TIME_INTERVAL
		 */	
		void SimulateStep();
		// Destructor
		~ProjectileSim();
};

		


