#pragma once

#include <array>

#include "../structure/Vector3D.h"

using namespace std;

// Time between each step of projectile simulation
#define TIME_INTERVAL 0.01

// Coefficient of drag
#define DRAG_COEF 0.47

// Cross secitonal area of fuel
#define AREA 0.0177

// Mass of the object
#define MASS 0.215

// Air density
#define AIR_DENSITY 1.225

// Downward force of gravity
#define GRAVITY 9.8

// Position of the center of the hub
#define HUB_POSITION Vector3D{4.6101, 4.03479, 1.4732}

// Positions to aim towards when passing
#define PASS_POSITIONS_0 Vector3D{1.8, 5.5, 0}
#define PASS_POSITIONS_1 Vector3D{1.8, 4, 0}
#define PASS_POSITIONS_2 Vector3D{1.8, 2.5, 0}
#define PASS_POSITIONS_3 Vector3D{0.5, 0.5, 0}

// Radius of the funnel on the hub
// Slightly smaller than real radius
#define FUNNEL_RADIUS 0.5

// Height of the funnel on the hub from the ground
#define FUNNEL_HEIGHT 1.8288

// Extra height to add in order to make sure fuel goes above obstacles
#define CLEARANCE 0.4

// Height of our shooter from the ground
#define SHOOTER_HEIGHT 0.5

// Height of the net from the ground
#define NET_HEIGHT 3.057144

// Y position of the right edge of the net
// Extra space added to account for the fact 
// that fuel is not just a single point
#define NET_RIGHT_EDGE HUB_POSITION.y + 0.741807 + 0.1

// Y position of the left edge of the net
// Extra space added to account for the fact 
// that fuel is not just a single point
#define NET_LEFT_EDGE HUB_POSITION.y - 0.741807 - 0.1

// X position of the net
#define NET_X_POS HUB_POSITION.x + 0.857504

// Height of the trench from the ground
#define TRENCH_HEIGHT 1.02235

// Size of the entire field, z coordinate is unsused
#define FIELD_SIZE Vector3D{16.54, 8.07, 0}

// Helper function that converts degrees to radians
#define ToRad(degrees) ( degrees * M_PI / 180 )

// Helper function that coverts radians to degrees
#define ToDeg(radians) ( radians * 180 / M_PI )

// NOTE: Shooter angles are expressed from the vertical axis, so lower angles = more steep trajectory

// Minimum shooter angle possible from z axis 
// We technically can go to 0 but in practice 
// we wouldn't want to shoot with an angle of zero
#define SHOOTER_MIN_ANGLE 10

// Maximum shooter angle possible from z axis
#define SHOOTER_MAX_ANGLE 60

// Maximum velocity the shooter can reach
#define SHOOTER_MAX_VELOCITY 15

