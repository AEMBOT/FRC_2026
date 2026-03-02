package com.aembot.lib.config.subsystems.flywheel;

import com.aembot.lib.config.motors.MotorConfiguration;
import com.aembot.lib.config.motors.SimulatedMotorConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import java.util.function.Function;

public class TalonFXFlywheelConfiguration {
  public final String kName;

  public MotorConfiguration<TalonFXConfiguration> kMotorConfig;

  public SimulatedMotorConfiguration<TalonFXConfiguration> kSimMotorConfig;

  /** The amount of units we can be off and still shoot */
  public double kAutoAimLeniance;

  /** The +- tolerance for the flywheel's speed to count as being at target in units per second */
  public Double kSpeedToleranceUnitsPerSecond = 0.2;

  /**
   * Function to simulate a load impulse, such as a game piece going through a shooter. Takes the
   * current velocity of the flywheel & returns the new velocity of the flywheel.
   *
   * <p><strong>Default:</strong>
   *
   * <pre>{@code (vel) -> vel}</pre>
   */
  public Function<Double, Double> kSimulateLoadImpulseFunction = (vel) -> vel;

  public TalonFXFlywheelConfiguration(String name) {
    this.kName = name;
  }

  public TalonFXFlywheelConfiguration withRealMotorConfig(
      MotorConfiguration<TalonFXConfiguration> realMotorConfig) {
    this.kMotorConfig = realMotorConfig;
    return this;
  }

  public TalonFXFlywheelConfiguration withSimulatedMotorConfig(
      SimulatedMotorConfiguration<TalonFXConfiguration> simMotorConfig) {
    this.kSimMotorConfig = simMotorConfig;
    return this;
  }

  /**
   * Set the amount of units that we can be off in order to still shoot
   *
   * <p>Counts both directions, so for example if this was 10, we could have a deviance of -10
   * through +10
   *
   * @return this {@link TalonFXFlywheelConfiguration} for chaining
   */
  public TalonFXFlywheelConfiguration withAutoAimLeniance(double autoAimLeniance) {
    this.kAutoAimLeniance = autoAimLeniance;
    return this;
  }

  /**
   * Set the function to simulate a load impulse, such as a game piece going through a shooter.
   * Takes the current velocity of the flywheel & returns the new velocity of the flywheel.
   *
   * <p><strong>Default:</strong>
   *
   * <pre>{@code (vel) -> vel}</pre>
   *
   * @return This {@link TalonFXFlywheelConfiguration} for chaining
   */
  public TalonFXFlywheelConfiguration withSimulateLoadImpulseFunction(
      Function<Double, Double> function) {
    this.kSimulateLoadImpulseFunction = function;
    return this;
  }

  /**
   * Set the +- tolerance for the flywheel's speed to count as being at target in units per second
   *
   * @return This {@link TalonFXFlywheelConfiguration} for chaining
   */
  public TalonFXFlywheelConfiguration withSpeedToleranceUnitsPerSecond(
      double speedToleranceUnitsPerSecond) {
    this.kSpeedToleranceUnitsPerSecond = speedToleranceUnitsPerSecond;
    return this;
  }
}
