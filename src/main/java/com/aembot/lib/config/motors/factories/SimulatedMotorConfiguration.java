package com.aembot.lib.config.motors.factories;

import com.aembot.lib.config.motors.MotorConfiguration;
import edu.wpi.first.math.system.plant.DCMotor;

public class SimulatedMotorConfiguration<T> {

  public MotorConfiguration<T> kRealConfiguration;

  public DCMotor kSimMotorConstants;

  public double kStartingRotationDegrees;

  public final SimulatedMotorConfiguration<T> withRealConfiguration(
      MotorConfiguration<T> realConfiguration) {
    this.kRealConfiguration = realConfiguration;
    return this;
  }

  public final SimulatedMotorConfiguration<T> withSimMotorConstants(DCMotor simMotorConstants) {
    this.kSimMotorConstants = simMotorConstants;
    return this;
  }

  public final SimulatedMotorConfiguration<T> withStartingRotations(
      double startingRotationDegrees) {
    this.kStartingRotationDegrees = startingRotationDegrees;
    return this;
  }
}
