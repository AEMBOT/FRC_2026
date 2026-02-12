package com.aembot.lib.config.motors;

import edu.wpi.first.math.system.plant.DCMotor;

public class SimulatedMotorConfiguration<T> {

  /** Real configuration to use */
  public MotorConfiguration<T> kRealConfiguration;

  /** The constants to use for the simulated motor */
  public DCMotor kSimMotorConstants;

  /** Starting rotation of the motor */
  public double kStartingRotationUnits;

  /**
   * @param realConfiguration Real configuration to use
   * @return This Simulated MotorConfiguration for method chaining
   */
  public final SimulatedMotorConfiguration<T> withRealConfiguration(
      MotorConfiguration<T> realConfiguration) {
    this.kRealConfiguration = realConfiguration;
    return this;
  }

  /**
   * @param simMotorConstants The constants to use for the simulated motor
   * @return This Simulated MotorConfiguration for method chaining
   */
  public final SimulatedMotorConfiguration<T> withSimMotorConstants(DCMotor simMotorConstants) {
    this.kSimMotorConstants = simMotorConstants;
    return this;
  }

  /**
   * @param startingRotationDegrees Starting rotation of the motor
   * @return This Simulated MotorConfiguration for method chaining
   */
  public final SimulatedMotorConfiguration<T> withStartingRotation(double startingRotationUnits) {
    this.kStartingRotationUnits = startingRotationUnits;
    return this;
  }
}
