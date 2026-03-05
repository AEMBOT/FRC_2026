#pragma once

#include <array>
#include <cmath>

#include "../structure/Vector3D.h"
#include "../data/Constants.h"

/**
 * Gives the two target positions to aim for if shooting given robot position
 *
 * @param robotPos position of the robot on the field
 * 
 * @return an array of positions where index 0 is the obstacle and index 1 is the goal
 */
array<Vector3D, 2> GetTargetPositionsShoot(Vector3D robotPos) {
	Vector3D obstacle;

	Vector3D robotToHub = HUB_POSITION - robotPos;

	obstacle = HUB_POSITION - ((robotToHub / robotToHub.mag()) * FUNNEL_RADIUS);

	obstacle.z = FUNNEL_HEIGHT + CLEARANCE;

	return {obstacle, HUB_POSITION};
}

/**
 * Gives the two target positions to aim for if passing given robot position
 *
 * @param robotPos position of the robot on the field
 * @param targetPosIndex index of the passing position to aim for, see Constants.h
 *
 * @return an array of positions where index 0 is the obstacle and index 1 is the goal
 */
array<Vector3D, 2> GetTargetPositionsPass(Vector3D robotPos, int targetPosIndex) {

	Vector3D passPos;
	
	switch (targetPosIndex) {
		case 0:
			passPos = PASS_POSITIONS_0;
			break;
		case 1:
			passPos = PASS_POSITIONS_1;
			break;
		case 2:
			passPos = PASS_POSITIONS_2;
			break;
		default:
			passPos = PASS_POSITIONS_3;
	};

	Vector3D passToObstacle = Vector3D{NET_X_POS, 0, 0} - passPos;

	Vector3D passToRobot = robotPos - passPos;  
	
	passToObstacle.y = (passToObstacle.x * passToRobot.y) / passToRobot.x;

	passToObstacle.z = NET_HEIGHT + CLEARANCE;

	if (passToObstacle.y + passPos.y < NET_RIGHT_EDGE && NET_LEFT_EDGE < passToObstacle.y + passPos.y) {
		return {passToObstacle + passPos, passPos};
	}

	passToObstacle = Vector3D{HUB_POSITION.x - passPos.x, 0, 0} - passPos;

	passToObstacle.y = (passToObstacle.x * passToRobot.y) / passToRobot.x;

	passToObstacle.z = TRENCH_HEIGHT + CLEARANCE;

	if (passToObstacle.y < NET_RIGHT_EDGE && NET_LEFT_EDGE < passToObstacle.y + passPos.y) {
		passToObstacle.z = NET_HEIGHT + CLEARANCE;
		return {passToObstacle + passPos, passPos};
	}

	return {passToObstacle + passPos, passPos};
}
