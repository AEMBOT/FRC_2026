package com.aembot.lib.config.motors;

import com.ctre.phoenix6.configs.CANcoderConfiguration;

/**
 * Configuration for a servo motor with a CANCoder
 *
 * <p>This contains config regarding the motor itself, how the motor should interact with the
 * CANCoder, and config for the CANCoder itself.
 *
 * @param <C> The type of the underlying configuration for the motor
 */
public class MotorCANCoderConfiguration<C> extends MotorConfiguration<C> {
  public CANcoderConfiguration canCoderConfiguration = new CANcoderConfiguration();

  /**
   * Gear ratio between the motor and the encoder; ratio of encoder rotations per motor rotation.
   */
  public Double encoderToMotorGearRatio = 1.0;

  /** If the CANCoder in use is fused */
  public boolean isFusedCANCoder = false;

  /**
   * Convert the given number of rotations to the units used for this motor using {@code
   * unitToCANCoderRotationRatio}, or {@code unitToRotationRatio} if not set
   */
  public double getEncoderRotationsToUnits(double rotorRotations) {
    return getRotationsToUnits(rotorRotations / encoderToMotorGearRatio);
  }

  /**
   * Take the given double in the units used for this motor, and convert it to rotations using
   * {@code unitToCANCoderRotationRatio}, or {@code unitToRotationRatio} if not set
   */
  public double getUnitsToEncoderRotations(double units) {
    return getUnitsToRotations(units / encoderToMotorGearRatio);
  }
}
