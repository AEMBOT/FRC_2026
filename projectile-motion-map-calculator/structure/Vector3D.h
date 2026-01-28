#pragma once

#include <cmath>

using namespace std;

// Helper struct that contains an x, y, z, value and many functions to manipulate them
struct Vector3D {

	double x, y, z;
	/** @return the rho value of this vector when converted to polar coordinates */
	double getRho() { return mag(); }

	/** @return the theta value of this vector when converted to polar coordinates */
	double getTheta() { return acos(z/mag()); }

	/** @return the phi value of this vector when converted to polar coordinates */
	double getPhi() { return atan2(y, x); }

	/** 
	 * create a new Vector3D from a given set of polar coordiates
	 *
	 * @param rho rho value of the new vector
	 * @param theta theta value of the new vector
	 * @param phi phi value of the new vector
	 *
	 * @return the created vector
	 */
	static Vector3D fromPolar(double rho, double theta, double phi) {
		return {
			rho * sin(theta) * cos(phi),
			rho * sin(theta) * sin(phi),
			rho * cos(theta),
		};
	}

	/**
	 * Set the rho value of this vector
	 *
	 * @param newRho the rho value to set
	 */
	void setRho(double newRho) {
		*this = fromPolar(newRho, getTheta(), getPhi());
	}

	/**
	 * Set the theta value of this vector
	 *
	 * @param newTheta the theta value to set
	 */
	void setTheta(double newTheta) {
		double rho = getRho();
		double phi = getPhi();
		*this = fromPolar(rho, newTheta, phi);
	}
	
	/**
	 * Set the phi value of this vector
	 *
	 * @param newPhi the phi value to set
	 */
	void setPhi(double newPhi) {
		*this = fromPolar(getRho(), getTheta(), newPhi);
	}	

	/**
	 * Change the rho value of this vector by a given amount
	 *
	 * @param deltaRho the amount to change rho by
	 */
	void changeRho(double deltaRho) {
		setRho(getRho() + deltaRho);
	}
	
	/**
	 * Change the theta value of this vector by a given amount
	 *
	 * @param deltaTheta the amount to change theta by
	 */
	void changeTheta(double deltaTheta) {
		setTheta(getTheta() + deltaTheta);
	}

	/**
	 * Change the phi value of this vector by a given amount
	 *
	 * @param deltaPhi the amount to change phi by
	 */
	void changePhi(double deltaPhi) {
		setPhi(getPhi() + deltaPhi);
	}

	/** @return the magnitude of this vector */
	double mag() {
		return sqrt(
				pow(x, 2) +
				pow(y, 2) +
				pow(z, 2) 
				);
	}

	/** 
	 * Calculates the distance between this vector an another
	 *
	 * @param other vector to get the distance to
	 *
	 * @return the magnitude of the difference between this and the other vector
	 */
	double distTo(Vector3D other) {
		return (other - *this).mag();
	}

	Vector3D operator+(const Vector3D& other) {
		Vector3D result;
		result.x = x + other.x;
		result.y = y + other.y;
		result.z = z + other.z;
		return result;
	}

	Vector3D operator-(const Vector3D& other) {
		Vector3D result;
		result.x = x - other.x;
		result.y = y - other.y;
		result.z = z - other.z;
		return result;
	}

	void operator+=(const Vector3D& other) {
		x += other.x;
		y += other.y;
		z += other.z;
	}

	Vector3D operator*(const double& other) {
		Vector3D result;
		result.x = x * other;
		result.y = y * other;
		result.z = z * other;
		return result;
	}

	Vector3D operator/(const double& other) {
		Vector3D result;
		result.x = x / other;
		result.y = y / other;
		result.z = z / other;
		return result;
	}


	Vector3D operator*(const int& other) {
		Vector3D result;
		result.x = x * other;
		result.y = y * other;
		result.z = z * other;
		return result;
	}

	friend ostream& operator<<(ostream& os,const Vector3D& vec) {
		os << vec.x << ", " << vec.y << ", " << vec.z;
		return os;
	}
};


