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
	ofstream cornerLeftFileReal("../src/main/deploy/initial-velocities/real/Passing_Left_Corner_Initial_Velocities.csv");
	ofstream cornerRightFileReal("../src/main/deploy/initial-velocities/real/Passing_Right_Corner_Initial_Velocities.csv");
	ofstream centerLeftFileReal("../src/main/deploy/initial-velocities/real/Passing_Left_Center_Initial_Velocities.csv");
	ofstream centerRightFileReal("../src/main/deploy/initial-velocities/real/Passing_Right_Center_Initial_Velocities.csv");

	ofstream hubFileSim("../src/main/deploy/initial-velocities/sim/Shooting_Hub_Initial_Velocities.csv");
	ofstream cornerLeftFileSim("../src/main/deploy/initial-velocities/sim/Passing_Left_Corner_Initial_Velocities.csv");
	ofstream cornerRightFileSim("../src/main/deploy/initial-velocities/sim/Passing_Right_Corner_Initial_Velocities.csv");
	ofstream centerLeftFileSim("../src/main/deploy/initial-velocities/sim/Passing_Left_Center_Initial_Velocities.csv");
	ofstream centerRightFileSim("../src/main/deploy/initial-velocities/sim/Passing_Right_Center_Initial_Velocities.csv");

	if (!hubFileReal.is_open()) {
		cout << "error opening hub file (real)" << endl;
		return 1;
	}

	if (!cornerLeftFileReal.is_open()) {
		cout << "error opening left file (real)" << endl;
		return 1;
	}
	
	if (!cornerRightFileReal.is_open()) {
		cout << "error opening mid file (real)" << endl;
		return 1;
	}

	if (!centerLeftFileReal.is_open()) {
		cout << "error opening right file (real)" << endl;
		return 1;
	}

	if (!centerRightFileReal.is_open()) {
		cout << "error opening outpost file (real)" << endl;
		return 1;
	}
	
	if (!hubFileSim.is_open()) {
		cout << "error opening hub file (sim)" << endl;
		return 1;
	}

	if (!cornerLeftFileSim.is_open()) {
		cout << "error opening left file (sim)" << endl;
		return 1;
	}
	
	if (!cornerRightFileSim.is_open()) {
		cout << "error opening mid file (sim)" << endl;
		return 1;
	}

	if (!centerLeftFileSim.is_open()) {
		cout << "error opening right file (sim)" << endl;
		return 1;
	}

	if (!centerRightFileSim.is_open()) {
		cout << "error opening outpost file (sim)" << endl;
		return 1;
	}
	
	const char* header = "X Position, Y Position, Z Position, X Velocity, Y Velocity, Z Velocity";

	hubFileReal << header << endl;
	cornerLeftFileReal << header << endl;
	cornerRightFileReal << header << endl;
	centerLeftFileReal << header << endl;
	centerRightFileReal << header << endl;
	hubFileSim << header << endl;
	cornerLeftFileSim << header << endl;
	cornerRightFileSim << header << endl;
	centerLeftFileSim << header << endl;
	centerRightFileSim << header << endl;

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
			if (point.error <= 5) {
				cornerLeftFileReal << robotPos << ", " << point.initVel << endl;
			}

			point = OptimizeTest(robotPos, GetTargetPositionsPass(robotPos, 1), PASSING, true);
			if (point.error <= 5) {
				cornerRightFileReal << robotPos << ", " << point.initVel << endl;
			}

			point = OptimizeTest(robotPos, GetTargetPositionsPass(robotPos, 2), PASSING, true);
			if (point.error <= 5) {
				centerLeftFileReal << robotPos << ", " << point.initVel << endl;
			}

			point = OptimizeTest(robotPos, GetTargetPositionsPass(robotPos, 3), PASSING, true);
			if (point.error <= 5) {
				centerRightFileReal << robotPos << ", " << point.initVel << endl;
			}
			
			point = OptimizeTest(robotPos, GetTargetPositionsShoot(robotPos), SHOOTING, false);
			if (point.error <= 0.5) {
				hubFileSim << robotPos << ", " << point.initVel << endl;
			}

			point = OptimizeTest(robotPos, GetTargetPositionsPass(robotPos, 0), PASSING, false);
			if (point.error <= 5) {
				cornerLeftFileSim << robotPos << ", " << point.initVel << endl;
			}

			point = OptimizeTest(robotPos, GetTargetPositionsPass(robotPos, 1), PASSING, false);
			if (point.error <= 5) {
				cornerRightFileSim << robotPos << ", " << point.initVel << endl;
			}

			point = OptimizeTest(robotPos, GetTargetPositionsPass(robotPos, 2), PASSING, false);
			if (point.error <= 5) {
				centerLeftFileSim << robotPos << ", " << point.initVel << endl;
			}

			point = OptimizeTest(robotPos, GetTargetPositionsPass(robotPos, 3), PASSING, false);
			if (point.error <= 5) {
				centerRightFileSim << robotPos << ", " << point.initVel << endl;
			}

		
		}
	}

	cout << "\r" << "                  " << "\r";
	cout.flush();
	printf("Calculating ... 100%%\n");
	cout.flush();


	hubFileReal.close();
	cornerLeftFileReal.close();
	cornerRightFileReal.close();
	centerLeftFileReal.close();
	centerRightFileReal.close();
	hubFileSim.close();
	cornerLeftFileSim.close();
	cornerRightFileSim.close();
	centerLeftFileSim.close();
	centerRightFileSim.close();

	auto end = chrono::high_resolution_clock::now();

	auto duration = chrono::duration_cast<chrono::microseconds>(end - start);

	cout << "Executed in " << (double)duration.count() / 1000000 << " seconds." << endl;

	return 0;

}

