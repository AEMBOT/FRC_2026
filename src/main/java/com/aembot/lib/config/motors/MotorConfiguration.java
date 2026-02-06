// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.aembot.lib.config.motors;

import com.aembot.lib.core.can.CANDeviceID;
import com.aembot.lib.core.motors.interfaces.MotorIO.NeutralMode;

/**
 * Configuration for treating a motor as effectively a servo
 *
 * <p>This allows for specifiying a unit conversion rate and min and max limits
 *
 * <p>T in this case is the underlying configuration for the motor
 */
public class MotorConfiguration<T> {

  // The actual configuration being used internally by the motor
  public T kMotorConfig = null;

  // The name of the configuration in use
  public String kConfigurationName = "UNNAMED";

  // The CAN device that is used by the device being configured
  public CANDeviceID kCANDevice = null;

  // Conversion factor for converting between the desired output units and rotations of the motor
  public double kUnitToRotorRotationRatio = 1.0;

  // In whatever units this configruation is using set the minimum position that this motor can
  // drive to
  public double kMinPositionUnits = Double.NEGATIVE_INFINITY;

  // In whatever units this configuration is using set the maximum position that this motor can
  // drive to
  public double kMaxPositionUnits = Double.POSITIVE_INFINITY;

  // Moment of Inertia (KgMetersSquared) (how resistant a motor's rotor is to changs in its
  // rotational speed)
  public double kMomentOfInertia = 0.5;

  // The neutral mode of the motor
  public NeutralMode kNeutralMode = NeutralMode.BRAKE;

  /**
   * Convert the current rotor rotations to the real-world units specified scaled by the
   * unitToRotorRotationRation defined in the config
   *
   * @param rotorRotations Rotor rotation value we want to convert into the in-use units
   * @return The rotations converted into some units as defined in the config
   */
  public double getRotorRotationsToUnits(double rotorRotations) {
    return rotorRotations * this.kUnitToRotorRotationRatio;
  }

  /**
   * Convert the current units into rotor rotations using the defined conversion ratio
   *
   * @param units Units we want to convert to rotor rotations
   * @return The resulting rotor rotations
   */
  public double getUnitsToRotorRotations(double units) {
    return units / this.kUnitToRotorRotationRatio;
  }

  /**
   * Get whatever the specified motor specific configuration was
   *
   * @return The motor configuration that this servo motor is using
   */
  public final T getMotorConfig() {
    return kMotorConfig;
  }

  /**
   * Update the configuration to some other config type
   *
   * @param config What motor configuration should we use under the hood
   * @return Reference to this motor configuration for chaining
   */
  public MotorConfiguration<T> withMotorConfig(T config) {
    this.kMotorConfig = config;
    return this;
  }

  /**
   * Update the name of the configuration that is in use
   *
   * @param name The new name to be set
   * @return Reference to this motor configuration for chaining
   */
  public MotorConfiguration<T> withName(String name) {
    this.kConfigurationName = name;
    return this;
  }

  /**
   * Update the name CANDeviceID used with this config
   *
   * @param device The CANDeviceID to set
   * @return Reference to this motor configuration for chaining
   */
  public MotorConfiguration<T> withCANDevice(CANDeviceID device) {
    this.kCANDevice = device;
    return this;
  }

  /**
   * Update the name unit to rotor ratio used with this config
   *
   * @param ratio The ratio to set
   * @return Reference to this motor configuration for chaining
   */
  public MotorConfiguration<T> withUnitToRotorRotationRatio(double ratio) {
    this.kUnitToRotorRotationRatio = ratio;
    return this;
  }

  /**
   * Update the minimum position limit in units
   *
   * @param min The minimum position in units
   * @return Reference to this motor configuration for chaining
   */
  public MotorConfiguration<T> withMinPositionUnits(double min) {
    this.kMinPositionUnits = min;
    return this;
  }

  /**
   * Update the maximum position limit in units
   *
   * @param max The maximum position in units
   * @return Reference to this motor configuration for chaining
   */
  public MotorConfiguration<T> withMaxPositionUnits(double max) {
    this.kMaxPositionUnits = max;
    return this;
  }

  /**
   * Update the moment of inertia (KgMetersSquared)
   *
   * @param moi The moment of inertia to set
   * @return Reference to this motor configuration for chaining
   */
  public MotorConfiguration<T> withMomentOfInertia(double moi) {
    this.kMomentOfInertia = moi;
    return this;
  }

  /**
   * Update the neutral mode of the motor
   *
   * @param mode The neutral mode to set
   * @return Reference to this motor configuration for chaining
   */
  public MotorConfiguration<T> withNeutralMode(NeutralMode mode) {
    this.kNeutralMode = mode;
    return this;
  }
}
