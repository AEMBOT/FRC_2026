#include <array>
#include <chrono>
#include <iostream>
#include <fstream>

#include "scripts/ProjectileSim.h"
#include "scripts/TrajectoryOptimizer.h"
#include "scripts/FieldPositionGenerator.h"
#include "structure/State.h"
#include "structure/Vector3D.h"

using namespace std;



TrialPoint OptimizeTest(Vector3D initPos, array<Vector3D, 2> goalPositions, optimizationType optimType, bool useDrag) {

	Vector3D p1 = goalPositions[0];
	Vector3D p2 = goalPositions[1];

	TrajectoryOptimizer optimizer = TrajectoryOptimizer(p1, p2, initPos, optimType, useDrag);

	optimizer.RunOptimizer();
	
	return TrialPoint{optimizer.GetBestVelocity(), optimizer.GetBestError()};

	}

int main() {
	auto start = chrono::high_resolution_clock::now();

	ofstream hubFileReal("../src/main/deploy/initial-velocities/real/Shooting_Hub_Initial_Velocities.csv");
	ofstream leftFileReal("../src/main/deploy/initial-velocities/real/Passing_Left_Initial_Velocities.csv");
	ofstream midFileReal("../src/main/deploy/initial-velocities/real/Passing_Middle_Initial_Velocities.csv");
	ofstream rightFileReal("../src/main/deploy/initial-velocities/real/Passing_Right_Initial_Velocities.csv");
	ofstream outpostFileReal("../src/main/deploy/initial-velocities/real/Passing_Outpost_Initial_Velocities.csv");

	ofstream hubFileSim("../src/main/deploy/initial-velocities/sim/Shooting_Hub_Initial_Velocities.csv");
	ofstream leftFileSim("../src/main/deploy/initial-velocities/sim/Passing_Left_Initial_Velocities.csv");
	ofstream midFileSim("../src/main/deploy/initial-velocities/sim/Passing_Middle_Initial_Velocities.csv");
	ofstream rightFileSim("../src/main/deploy/initial-velocities/sim/Passing_Right_Initial_Velocities.csv");
	ofstream outpostFileSim("../src/main/deploy/initial-velocities/sim/Passing_Outpost_Initial_Velocities.csv");

	if (!hubFileReal.is_open()) {
		cout << "error opening hub file (real)" << endl;
		return 1;
	}

	if (!leftFileReal.is_open()) {
		cout << "error opening left file (real)" << endl;
		return 1;
	}
	
	if (!midFileReal.is_open()) {
		cout << "error opening mid file (real)" << endl;
		return 1;
	}

	if (!rightFileReal.is_open()) {
		cout << "error opening right file (real)" << endl;
		return 1;
	}

	if (!outpostFileReal.is_open()) {
		cout << "error opening outpost file (real)" << endl;
		return 1;
	}
	
	if (!hubFileSim.is_open()) {
		cout << "error opening hub file (sim)" << endl;
		return 1;
	}

	if (!leftFileSim.is_open()) {
		cout << "error opening left file (sim)" << endl;
		return 1;
	}
	
	if (!midFileSim.is_open()) {
		cout << "error opening mid file (sim)" << endl;
		return 1;
	}

	if (!rightFileSim.is_open()) {
		cout << "error opening right file (sim)" << endl;
		return 1;
	}

	if (!outpostFileSim.is_open()) {
		cout << "error opening outpost file (sim)" << endl;
		return 1;
	}
	
	const char* header = "X Position, Y Position, Z Position, X Velocity, Y Velocity, Z Velocity";

	hubFileReal << header << endl;
	leftFileReal << header << endl;
	midFileReal << header << endl;
	rightFileReal << header << endl;
	outpostFileReal << header << endl;
	hubFileSim << header << endl;
	leftFileSim << header << endl;
	midFileSim << header << endl;
	rightFileSim << header << endl;
	outpostFileSim << header << endl;

	int totalIter = ceil((FIELD_SIZE.x / 0.25)) * ceil((FIELD_SIZE.y / 0.25));

	int numIter = 0;

	for (double i = 0; i <= FIELD_SIZE.x; i += 0.25) {
		for (double j = 0; j <= FIELD_SIZE.y; j += 0.25) {

			cout << "\r" << "                  " << "\r";
			cout.flush();
			printf("Calculating ... %d%%", ((numIter++ * 100) / totalIter));
			cout.flush();


			Vector3D robotPos = {i, j, SHOOTER_HEIGHT};
			TrialPoint point;

			point = OptimizeTest(robotPos, GetTargetPositionsShoot(robotPos), SHOOTING, true);
			if (point.error <= 0.5) {
				hubFileReal << robotPos << ", " << point.initVel << endl;
			}

			point = OptimizeTest(robotPos, GetTargetPositionsPass(robotPos, 0), PASSING, true);
			if (point.error <= 0.5) {
				leftFileReal << robotPos << ", " << point.initVel << endl;
			}

			point = OptimizeTest(robotPos, GetTargetPositionsPass(robotPos, 1), PASSING, true);
			if (point.error <= 0.5) {
				midFileReal << robotPos << ", " << point.initVel << endl;
			}

			point = OptimizeTest(robotPos, GetTargetPositionsPass(robotPos, 2), PASSING, true);
			if (point.error <= 0.5) {
				rightFileReal << robotPos << ", " << point.initVel << endl;
			}

			point = OptimizeTest(robotPos, GetTargetPositionsPass(robotPos, 3), PASSING, true);
			if (point.error <= 0.5) {
				outpostFileReal << robotPos << ", " << point.initVel << endl;
			}
			
			point = OptimizeTest(robotPos, GetTargetPositionsShoot(robotPos), SHOOTING, false);
			if (point.error <= 0.5) {
				hubFileSim << robotPos << ", " << point.initVel << endl;
			}

			point = OptimizeTest(robotPos, GetTargetPositionsPass(robotPos, 0), PASSING, false);
			if (point.error <= 0.5) {
				leftFileSim << robotPos << ", " << point.initVel << endl;
			}

			point = OptimizeTest(robotPos, GetTargetPositionsPass(robotPos, 1), PASSING, false);
			if (point.error <= 0.5) {
				midFileSim << robotPos << ", " << point.initVel << endl;
			}

			point = OptimizeTest(robotPos, GetTargetPositionsPass(robotPos, 2), PASSING, false);
			if (point.error <= 0.5) {
				rightFileSim << robotPos << ", " << point.initVel << endl;
			}

			point = OptimizeTest(robotPos, GetTargetPositionsPass(robotPos, 3), PASSING, false);
			if (point.error <= 0.5) {
				outpostFileSim << robotPos << ", " << point.initVel << endl;
			}

		
		}
	}

	cout << "\r" << "                  " << "\r";
	cout.flush();
	printf("Calculating ... 100%%\n");
	cout.flush();


	hubFileReal.close();
	leftFileReal.close();
	midFileReal.close();
	rightFileReal.close();
	outpostFileReal.close();
	hubFileSim.close();
	leftFileSim.close();
	midFileSim.close();
	rightFileSim.close();
	outpostFileSim.close();

	auto end = chrono::high_resolution_clock::now();

	auto duration = chrono::duration_cast<chrono::microseconds>(end - start);

	cout << "Executed in " << (double)duration.count() / 1000000 << " seconds." << endl;

	return 0;

}

