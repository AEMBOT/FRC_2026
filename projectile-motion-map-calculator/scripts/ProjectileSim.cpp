#include <iostream>
#include <cmath>
#include "../structure/Vector3D.h"
#include "../structure/State.h"
#include "ProjectileSim.h"

using namespace std;

ProjectileSim::ProjectileSim(Vector3D initPos, Vector3D initVel) {
	projectileState = {initPos, initVel};
}

void ProjectileSim::SetProjectileState(State inState) {
	projectileState = inState;
}

State ProjectileSim::GetProjectileState() {
	return projectileState;
}

void ProjectileSim::SimulateStep() {

	State k1_s = projectileState;

	Vector3D k1_p = k1_s.position;
	Vector3D k1_v = k1_s.velocity;
	Vector3D k1_a = k1_s.getAcceleration();

	State k2_s = {k1_p + k1_v * 0.5 * TIME_INTERVAL,
				  k1_v + k1_a * 0.5 * TIME_INTERVAL}; 

	Vector3D k2_p = k2_s.position;
	Vector3D k2_v = k2_s.velocity;
	Vector3D k2_a = k2_s.getAcceleration();

	State k3_s = {k2_p + k2_v * 0.5 * TIME_INTERVAL,
				  k2_v + k2_a * 0.5 * TIME_INTERVAL}; 

	Vector3D k3_p = k3_s.position;
	Vector3D k3_v = k3_s.velocity;
	Vector3D k3_a = k3_s.getAcceleration();

	State k4_s = {k3_p + k2_v * 0.5 * TIME_INTERVAL,
				  k3_v + k3_a * 0.5 * TIME_INTERVAL}; 

	Vector3D k4_p = k4_s.position;
	Vector3D k4_v = k4_s.velocity;
	Vector3D k4_a = k4_s.getAcceleration();

	projectileState.position += (k1_v + k2_v*2 + k3_v*2 + k4_v) * (TIME_INTERVAL/6);
	projectileState.velocity += (k1_a + k2_a*2 + k3_a*2 + k4_a) * (TIME_INTERVAL/6);
}

ProjectileSim::~ProjectileSim() {}

