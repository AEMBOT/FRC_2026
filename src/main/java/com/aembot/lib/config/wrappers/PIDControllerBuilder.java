package com.aembot.lib.config.wrappers;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * Class for storing PID Controller constants (P, I, D, F, IZone, and output limits) and
 * initializing a {@link PIDController} with them. This class uses the builder pattern to facilitate
 * easy and readable initialization.
 */
public class PIDControllerBuilder {
  public Double p = null;
  public Double i = null;
  public Double d = null;
  public Double iZone = null; // Integral zone limit
  public Pair<Double, Double> continuousInput =
      null; // Will be non-null if continuous input is enabled
  public Double tolerance = null; // Output Tolerance

  /**
   * Sets the Proportional Gain (kP).
   *
   * @param p The proportional gain value.
   * @return The PIDControllerBuilder instance for chaining.
   */
  public PIDControllerBuilder withP(double p) {
    this.p = p;
    return this;
  }

  /**
   * Sets the Integral Gain (kI).
   *
   * @param i The integral gain value.
   * @return The PIDControllerBuilder instance for chaining.
   */
  public PIDControllerBuilder withI(double i) {
    this.i = i;
    return this;
  }

  /**
   * Sets the Derivative Gain (kD).
   *
   * @param d The derivative gain value.
   * @return The PIDControllerBuilder instance for chaining.
   */
  public PIDControllerBuilder withD(double d) {
    this.d = d;
    return this;
  }

  /**
   * Tolerance to give the PID Controller
   *
   * @param tolerance How much tolerance to grant the controller.
   * @return The PIDControllerBuilder instance for chaining.
   */
  public PIDControllerBuilder withTolerance(double tolerance) {
    this.tolerance = tolerance;
    return this;
  }

  /**
   * Sets the Integral Zone (IZone or I-Range).
   *
   * @param iZone The integral zone value.
   * @return The PIDControllerBuilder instance for chaining.
   */
  public PIDControllerBuilder withIZone(double iZone) {
    this.iZone = iZone;
    return this;
  }

  /**
   * Sets the continuous input to the desired range
   *
   * @param lowerBound Lower bound of the continuous input
   * @param upperBound Upper bound of the continuous input
   * @return The PIDControllerBuilder instance for chaining.
   */
  public PIDControllerBuilder withContinuousInput(double lowerBound, double upperBound) {
    this.continuousInput = new Pair<Double, Double>(lowerBound, upperBound);
    return this;
  }

  /**
   * Generate a new PID controller from the given controller constants
   *
   * @return A newly created PID controller
   */
  public PIDController generateController() {
    if (p != null && i != null && d != null) {
      PIDController controller = new PIDController(p, i, d);

      // Set I-zone if supplied
      if (iZone != null) {
        controller.setIZone(iZone);
      }

      // Set tolerance if supplied
      if (tolerance != null) {
        controller.setTolerance(tolerance);
      }

      // Enable continuous input if supplied
      if (continuousInput != null) {
        controller.enableContinuousInput(continuousInput.getFirst(), continuousInput.getSecond());
      }

      return controller;
    } else {
      DriverStation.reportError(
          "Attempted to generate PID controller where P, I, or D gains were unset", true);
      return null;
    }
  }
}
