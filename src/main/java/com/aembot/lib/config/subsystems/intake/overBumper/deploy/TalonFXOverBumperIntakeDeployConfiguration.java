package com.aembot.lib.config.subsystems.intake.overBumper.deploy;

import com.aembot.lib.config.motors.MotorConfiguration;
import com.aembot.lib.config.motors.SimulatedMotorConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.geometry.Pose3d;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;

public class TalonFXOverBumperIntakeDeployConfiguration {

  public final String kName;

  /** Motor configuration for real subsystem */
  public MotorConfiguration<TalonFXConfiguration> kRealMotorConfig;

  /** Motor configuration for simulated subsystem */
  public SimulatedMotorConfiguration<TalonFXConfiguration> kSimMotorConfig;

  /** Speed at which to run the motor while zeroing, in degrees per second. Used in sim */
  public double kZeroingSpeedDegPerSec;

  /** The width of the intake in meters when deployed. Used in sim. */
  public double kWidthMeters;

  /**
   * The extension of the intake out of the drivetrain frame when deployed in meters. Used in sim.
   */
  public double kExtensionMeters;

  /** The side of the robot that the intake is on. Used in sim. */
  public IntakeSide kSide;

  /**
   * The origin point of the intake deploy mechanism. Used for advantagescope mechanism
   * visualization.
   */
  public Pose3d kPivotPoint;

  public TalonFXOverBumperIntakeDeployConfiguration(String name) {
    this.kName = name;
  }

  public TalonFXOverBumperIntakeDeployConfiguration withSimulatedMotorConfiguration(
      SimulatedMotorConfiguration<TalonFXConfiguration> simMotorConfig) {
    this.kSimMotorConfig = simMotorConfig;
    return this;
  }

  public TalonFXOverBumperIntakeDeployConfiguration withRealMotorConfiguration(
      MotorConfiguration<TalonFXConfiguration> realMotorConfig) {
    this.kRealMotorConfig = realMotorConfig;
    return this;
  }

  /**
   * @param zeroingSpeed Speed at which to run the motor while zeroing, in degrees per second
   * @return A reference to this object for chaining
   */
  public TalonFXOverBumperIntakeDeployConfiguration withZeroingSpeed(double zeroingSpeed) {
    this.kZeroingSpeedDegPerSec = zeroingSpeed;
    return this;
  }

  /**
   * Set the width of the intake in meters. Used in sim.
   *
   * @return A reference to this object for chaining
   */
  public TalonFXOverBumperIntakeDeployConfiguration withWidthMeters(double widthMeters) {
    this.kWidthMeters = widthMeters;
    return this;
  }

  /**
   * Set the extension of the intake out of the drivetrain frame when deployed in meters. Used in
   * sim.
   *
   * @return A reference to this object for chaining
   */
  public TalonFXOverBumperIntakeDeployConfiguration withExtensionMeters(double extensionMeters) {
    this.kExtensionMeters = extensionMeters;
    return this;
  }

  /**
   * Set the side of the robot that the intake is on. Used in sim.
   *
   * @return A reference to this object for chaining
   */
  public TalonFXOverBumperIntakeDeployConfiguration withIntakeSide(IntakeSide side) {
    this.kSide = side;
    return this;
  }

  /**
   * Set the origin point of the intake deploy mechanism. Used for advantagescope mechanism
   * visualization.
   *
   * @return A reference to this object for chaining
   */
  public TalonFXOverBumperIntakeDeployConfiguration withPivotPoint(Pose3d pivotPoint) {
    this.kPivotPoint = pivotPoint;
    return this;
  }
}
