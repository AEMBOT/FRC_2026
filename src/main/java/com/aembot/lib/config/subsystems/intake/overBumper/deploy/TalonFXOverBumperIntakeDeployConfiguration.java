package com.aembot.lib.config.subsystems.intake.overBumper.deploy;

import com.aembot.lib.config.motors.MotorConfiguration;
import com.aembot.lib.config.motors.SimulatedMotorConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class TalonFXOverBumperIntakeDeployConfiguration {

  public final String kName;

  /** Motor configuration for real subsystem */
  public MotorConfiguration<TalonFXConfiguration> kRealMotorConfig;

  /** Motor configuration for simulated subsystem */
  public SimulatedMotorConfiguration<TalonFXConfiguration> kSimMotorConfig;

  /** The angle at which the deploy motor stops based on the upwards hard stop */
  public double kUpwardStopAngle;

  /** The angle at which the deploy motor stops based on the sownwards hard stop */
  public double kDownwardStopAngle;

  /** Speed at which to run the motor while zeroing, in degrees per second */
  public double kZeroingSpeedDegPerSec;

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
   * @param upwardStopAngle The angle at which the deploy motor stops based on the upwards hard stop
   * @return A reference to this object for chaining
   */
  public TalonFXOverBumperIntakeDeployConfiguration withUpwardStopAngle(double upwardStopAngle) {
    this.kUpwardStopAngle = upwardStopAngle;
    return this;
  }

  /**
   * @param upwardStopAngle The angle at which the deploy motor stops based on the downwards hard
   *     stop
   * @return A reference to this object for chaining
   */
  public TalonFXOverBumperIntakeDeployConfiguration withDownwardStopAngle(
      double downwardStopAngle) {
    this.kDownwardStopAngle = downwardStopAngle;
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
}
