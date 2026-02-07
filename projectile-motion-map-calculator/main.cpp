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



TrialPoint OptimizeTest(Vector3D initPos, array<Vector3D, 2> goalPositions, optimizationType optimType) {

	Vector3D p1 = goalPositions[0];
	Vector3D p2 = goalPositions[1];

	TrajectoryOptimizer optimizer = TrajectoryOptimizer(p1, p2, initPos, optimType);

	optimizer.RunOptimizer();
	
	return TrialPoint{optimizer.GetBestVelocity(), optimizer.GetBestError()};

	}

int main() {
	auto start = chrono::high_resolution_clock::now();

	ofstream hubFile("../src/main/deploy/initial-velocities/Shooting_Hub_Initial_Velocities.csv");
	ofstream leftFile("../src/main/deploy/initial-velocities/Passing_Left_Initial_Velocities.csv");
	ofstream midFile("../src/main/deploy/initial-velocities/Passing_Middle_Initial_Velocities.csv");
	ofstream rightFile("../src/main/deploy/initial-velocities/Passing_Right_Initial_Velocities.csv");
	ofstream outpostFile("../src/main/deploy/initial-velocities/Passing_Outpost_Initial_Velocities.csv");

	if (!hubFile.is_open()) {
		cout << "error opening hub file" << endl;
		return 1;
	}

	if (!leftFile.is_open()) {
		cout << "error opening left file" << endl;
		return 1;
	}
	
	if (!midFile.is_open()) {
		cout << "error opening mid file" << endl;
		return 1;
	}

	if (!rightFile.is_open()) {
		cout << "error opening right file" << endl;
		return 1;
	}

	if (!outpostFile.is_open()) {
		cout << "error opening outpost file" << endl;
		return 1;
	}
	
	const char* header = "X Position, Y Position, Z Position, X Velocity, Y Velocity, Z Velocity";

	hubFile << header << endl;
	leftFile << header << endl;
	midFile << header << endl;
	rightFile << header << endl;
	outpostFile << header << endl;

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

			point = OptimizeTest(robotPos, GetTargetPositionsShoot(robotPos), SHOOTING);
			if (point.error <= 0.5) {
				hubFile << robotPos << ", " << point.initVel << endl;
			}

			point = OptimizeTest(robotPos, GetTargetPositionsPass(robotPos, 0), PASSING);
			if (point.error <= 0.5) {
				leftFile << robotPos << ", " << point.initVel << endl;
			}

			point = OptimizeTest(robotPos, GetTargetPositionsPass(robotPos, 1), PASSING);
			if (point.error <= 0.5) {
				midFile << robotPos << ", " << point.initVel << endl;
			}

			point = OptimizeTest(robotPos, GetTargetPositionsPass(robotPos, 2), PASSING);
			if (point.error <= 0.5) {
				rightFile << robotPos << ", " << point.initVel << endl;
			}

			point = OptimizeTest(robotPos, GetTargetPositionsPass(robotPos, 3), PASSING);
			if (point.error <= 0.5) {
				outpostFile << robotPos << ", " << point.initVel << endl;
			}
		}
	}

	cout << "\r" << "                  " << "\r";
	cout.flush();
	printf("Calculating ... 100%%\n");
	cout.flush();


	hubFile.close();
	leftFile.close();
	midFile.close();
	rightFile.close();
	outpostFile.close();

	auto end = chrono::high_resolution_clock::now();

	auto duration = chrono::duration_cast<chrono::microseconds>(end - start);

	cout << "Executed in " << (double)duration.count() / 1000000 << " seconds." << endl;

	return 0;

}

