#include <iostream>
#include <future>
#include <array>
#include <vector>
#include <cmath>

#include "ProjectileSim.h"
#include "TrajectoryOptimizer.h"
#include "../structure/Vector3D.h"
#include "../data/Constants.h"

using namespace std;

TrajectoryOptimizer::TrajectoryOptimizer(Vector3D in_obsPos, Vector3D in_goalPos, Vector3D in_initPos, optimizationType in_optimType) {
	obsPos = in_obsPos;
	goalPos = in_goalPos;
	initPos = in_initPos;
	optimType = in_optimType;

	minSearchSize = 0.0001;

	bestPoint = TrialPoint{Vector3D{0, 0, 0}, INFINITY};
}

TrajectoryOptimizer::~TrajectoryOptimizer() {}

TrialPoint TrajectoryOptimizer::objective(Vector3D initVel) {
	if (optimType == SHOOTING) {
		return objectiveHub(initVel);
	} else {
		return objectivePass(initVel);
	}
}


TrialPoint TrajectoryOptimizer::objectiveHub(Vector3D initVel) {

	ProjectileSim sim = ProjectileSim(initPos, initVel);

	Vector3D lastPos = {0, 0, 0};

	double minObsDist = INFINITY;
	double minGoalDist = INFINITY;

	bool inFunnel = false;

	Vector3D pPos = sim.GetProjectileState().position;


	while (sim.GetProjectileState().position.z >= 0) {

		sim.SimulateStep();

		pPos = sim.GetProjectileState().position;

		double obsDist = pPos.distTo(obsPos);
		double goalDist = pPos.distTo(goalPos);

		if (obsDist < minObsDist) { minObsDist = obsDist; }
		if (goalDist < minGoalDist) { minGoalDist = goalDist; }

		double horizDistToHub = sqrt(pow(HUB_POSITION.x - pPos.x, 2) + pow(HUB_POSITION.y - pPos.y, 2));

		if (lastPos.z > FUNNEL_HEIGHT &&
			horizDistToHub < FUNNEL_RADIUS &&
			pPos.z < FUNNEL_HEIGHT) {
			inFunnel = true;
		}

		lastPos = pPos;
	}

	if (initVel.getTheta() < ToRad(SHOOTER_MIN_ANGLE) || initVel.getTheta() > ToRad(SHOOTER_MAX_ANGLE) || initVel.getRho() > SHOOTER_MAX_VELOCITY) {
		return TrialPoint{initVel, INFINITY};
	} else if (inFunnel) {	
		return TrialPoint{initVel, pow(minObsDist, 2) + pow(minGoalDist, 2)};
	} else {
		return TrialPoint{initVel, INFINITY};
	}
}

TrialPoint TrajectoryOptimizer::objectivePass(Vector3D initVel) {

	ProjectileSim sim = ProjectileSim(initPos, initVel);

	Vector3D lastPos = {0, 0, 0};

	double minObsDist = INFINITY;
	double minGoalDist = INFINITY;

	bool overObs = true;

	Vector3D pPos = sim.GetProjectileState().position;

	while (sim.GetProjectileState().position.z >= 0) {

		sim.SimulateStep();

		pPos = sim.GetProjectileState().position;

		double obsDist = pPos.distTo(obsPos);
		double goalDist = pPos.distTo(goalPos);

		if (obsDist < minObsDist) { minObsDist = obsDist; }
		if (goalDist < minGoalDist) { minGoalDist = goalDist; }


		if (lastPos.x > obsPos.x &&
			pPos.x < obsPos.x &&
			pPos.z < obsPos.z - CLEARANCE) {
			overObs = false;
		}

		lastPos = pPos;
	}

	if (pPos.x > obsPos.x) { overObs = false; }

	if (initVel.getTheta() < ToRad(SHOOTER_MIN_ANGLE) || initVel.getTheta() > ToRad(SHOOTER_MAX_ANGLE) || initVel.getRho() > SHOOTER_MAX_VELOCITY) {
		return TrialPoint{initVel, INFINITY};
	} else if (overObs) {	
		return TrialPoint{initVel, pow(minObsDist, 2) + pow(minGoalDist, 2)};
	} else {
		return TrialPoint{initVel, INFINITY};
	}
}





TrialPoint TrajectoryOptimizer::PatternSearchRecursive(double searchSize, TrialPoint bestPoint) {

	TrialPoint newBest = bestPoint;

	testPoints[0] = bestPoint.initVel;
	testPoints[0].changeRho(searchSize * 5);

	testPoints[1] = bestPoint.initVel;
	testPoints[1].changeRho(-searchSize * 5);

	testPoints[2] = bestPoint.initVel;
	testPoints[2].changeTheta(searchSize * ToRad(90));

	testPoints[3] = bestPoint.initVel;
	testPoints[3].changeTheta(-searchSize * ToRad(90));

	for (int i = 0; i < 4; ++i) {
		threads[i] = async(launch::async, [this](Vector3D p) { return objective(p); }, testPoints[i]);
	}

	for (int i = 0; i < 4; ++i) {
		double error = threads[i].get().error;
		if (error < bestPoint.error) { newBest = TrialPoint{testPoints[i], error}; }
	}

	if (newBest.error == bestPoint.error) { searchSize *= 0.75; } 

	if (searchSize < minSearchSize) { return newBest; }

	return PatternSearchRecursive(searchSize, newBest);
}

void TrajectoryOptimizer::RunOptimizer() {

	Vector3D initGuessVel = goalPos - initPos;

	initGuessVel.setRho(0.5);

	TrialPoint initGuess = {initGuessVel, INFINITY};

	double initPhi = initGuess.initVel.getPhi();

	while (initGuess.error == INFINITY && initGuessVel.getRho() < SHOOTER_MAX_VELOCITY) {

		for (double theta = SHOOTER_MIN_ANGLE; theta < SHOOTER_MAX_ANGLE; theta += 1) {

			initGuessVel.setTheta(ToRad(theta));

			initGuessVel.setPhi(initPhi);

			TrialPoint testPoint = objective(initGuessVel);

			if (testPoint.error < initGuess.error) { initGuess = testPoint;}
		}

		initGuessVel.changeRho(1);
	}

	if (initGuess.error == INFINITY) { bestPoint = initGuess; }

	bestPoint = PatternSearchRecursive(4, initGuess);

	if (bestPoint.error > initGuess.error) {
		bestPoint = initGuess;
	}

	//cout << bestPoint.initVel.getRho() << ", ";
	//cout << ToDeg(bestPoint.initVel.getTheta()) << ", ";
	//cout << ToDeg(bestPoint.initVel.getPhi()) << endl;

}	

Vector3D TrajectoryOptimizer::GetBestVelocity() {
	return bestPoint.initVel;
}

double TrajectoryOptimizer::GetBestError() {
	return bestPoint.error;
}


