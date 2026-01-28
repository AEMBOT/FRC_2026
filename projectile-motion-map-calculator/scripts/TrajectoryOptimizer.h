#pragma once

#include <iostream>
#include <future>
#include <array>
	
#include "../structure/Vector3D.h"
#include "../structure/State.h"
#include "../data/Constants.h"

using namespace std;

// Helper struct to connect an initial velocity to an error value 
struct TrialPoint {
	Vector3D initVel;
	double error;
};

// Enumerator to switch between which kind of trajectory we are optimizing for
// Changes behavior when checking if we made it over obstacle
enum optimizationType {
	SHOOTING,
	PASSING
};

class TrajectoryOptimizer {
	private:
		// points to test for the objective function
		array<Vector3D, 4> testPoints;
		// threads to use during objective function
		array<future<TrialPoint>, 4> threads;
		// position of the first target position
		// this is the one used to clear the obstacle (funnel / net / trench)
		Vector3D obsPos;
		// position of the second target position
		// this is the one used to ensure that we end at the correct place
		Vector3D goalPos;
		// inital position of the projectile
		Vector3D initPos;
		// minimum search size before value is reasonably optimal
		double minSearchSize;
		// best point found
		TrialPoint bestPoint;
		// The kind of optimization to use (shooting / passing)
		optimizationType optimType;

		/**
		 * Recursive pattern search optimization function
		 *
		 * multithreaded for better performance
		 *
		 * @param searchSize distance in state space to search orthogonal direction
		 * @param bestPoint the initial velocity + error pair with the lowest error so far
		 *
		 * @return initial velocity pair found when search size < minSearchSize
		 */ 
		TrialPoint PatternSearchRecursive(double searchSize, TrialPoint bestPoint);

		/**
		 * objective function for the supplied initial condition
		 * this function the calles the more specific functions based 
		 * on whether we are shooting or passing
		 *
		 * @param initVel initial velocity of the projectile
		 *
		 * @return initial velocity paired with its corresponding error value
		 */
		TrialPoint objective(Vector3D initVel);

		/**
		 * objective function for the supplied initial condition
		 *
		 * Calculated the closest distance to the target positions
		 * returns the error as the sum of those distances squared
		 *
		 * error is set to infinity if we dont go into the hub or 
		 * if the initial velocuty is out of our bots capabilities
		 *
		 * @param initVel initial velocity of the projectile
		 *
		 * @return initial velocity paired with its corresponding error value
		 */
		TrialPoint objectiveHub(Vector3D initVel);
		
		/**
		 * objective function for the supplied initial condition
		 *
		 * Calculated the closest distance to the target positions
		 * returns the error as the sum of those distances squared
		 *
		 * error is set to infinity if we dont go over the obstacle or 
		 * if the initial velocuty is out of our bots capabilities
		 *
		 * @param initVel initial velocity of the projectile
		 *
		 * @return initial velocity paired with its corresponding error value
		 */
		TrialPoint objectivePass(Vector3D initVel);


	public:
		/**
		 * Class that will optimize the initial velocity to pass through two target points
		 *
		 * @param in_obsPos the first target position to optimize for
		 * @param in_goalPos the second target position to optimize for
		 * @param in_initPos the initial position of the projectile
		 * @param in_optimType kind of trajectory to optimize for (passing / shooting)
		 */
		TrajectoryOptimizer(Vector3D in_obsPos, Vector3D in_goalPos, Vector3D in_initPos, optimizationType in_optimType);

		~TrajectoryOptimizer();

		/**
		 * Runs the optimization algorithm
		 */
		void RunOptimizer();

		/**
		 * @return the velocity with the lowest corresponding error value
		 */
		Vector3D GetBestVelocity();

		/**
		 * @return the lowest error value found
		 */
		double GetBestError();
};
