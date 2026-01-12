package com.aembot.lib.config.motors;

import com.aembot.lib.core.can.CANDeviceID;

/**
 * Configuration for treating a motor as effectively a servo
 *
 * <p>This allows for specifiying a unit conversion rate and min and max limits
 *
 * @param <C> The type of the underlying configuration for the motor
 */
public class MotorConfiguration<C> {
  /** Configuration passed to the motor. */
  public C motorConfig = null;

  public String configurationName = "UNNAMED";

  /** The CAN device representation of the motor being configured */
  public CANDeviceID canDevice = null;

  /**
   * Conversion factor for converting between the desired output units and rotations of the motor
   */
  public double unitToRotationRatio = 1;

  /** The min position that this motor can drive to, in the unit of this motor */
  public double minPositionUnits = Double.NEGATIVE_INFINITY;

  /** The max position that this motor can drive to, in the unit of this motor */
  public double maxPositionUnits = Double.POSITIVE_INFINITY;

  // Moment of Inertia (KgMetersSquared) (how resistant a motor's rotor is to changs in its
  // rotational speed)
  public double momentOfInertia = 0.5;

  /**
   * Update the reference to the underlying motor configuration.
   *
   * @return Reference to this motor configuration for chaining
   */
  public MotorConfiguration<C> withConfig(C config) {
    this.motorConfig = config;
    return this;
  }

  /**
   * Update the name of this motor configurations
   *
   * @return Reference to this motor configuration for chaining
   */
  public MotorConfiguration<C> withName(String name) {
    this.configurationName = name;
    return this;
  }

  /**
   * Update the CANDeviceID used with this config
   *
   * @param device The CANDeviceID to set
   * @return Reference to this motor configuration for chaining
   */
  public MotorConfiguration<C> withCANDevice(CANDeviceID device) {
    this.canDevice = device;
    return this;
  }

  /**
   * Convert the given number of rotations to the units used for this motor using {@code
   * unitToRotationRatio}
   */
  public double getRotationsToUnits(double rotations) {
    return rotations * unitToRotationRatio;
  }

  /**
   * Take the given double in the units used for this motor, and convert it to rotations using
   * {@code unitToRotationRatio}
   */
  public double getUnitsToRotations(double units) {
    return units / unitToRotationRatio;
  }

  /** Get the provided reference to the underlying motor configuration. */
  public C getMotorConfig() {
    return motorConfig;
  }
}
