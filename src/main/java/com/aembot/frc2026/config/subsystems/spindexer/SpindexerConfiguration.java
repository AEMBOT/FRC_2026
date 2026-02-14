package com.aembot.frc2026.config.subsystems.spindexer;

import com.aembot.lib.config.motors.MotorConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import java.util.ArrayList;
import java.util.List;

public class SpindexerConfiguration {
  public final String kName;
  public final MotorConfiguration<TalonFXConfiguration> kMotorConfig;

  public Double kTargetSpeedRPM;

  public SpindexerConfiguration(String name, MotorConfiguration<TalonFXConfiguration> motorConfig) {
    this.kMotorConfig = motorConfig;
    this.kName = name;
  }

  /**
   * Set the target RPM of the spindexer while running
   *
   * @param targetSpeedRPM target RPM of the spindexer while running
   * @return this {@link SpindexerConfiguration} for chaining
   */
  public SpindexerConfiguration withTargetSpeedRPM(double targetSpeedRPM) {
    this.kTargetSpeedRPM = targetSpeedRPM;
    return this;
  }

  /**
   * Check that all values required for a spindexer subsystem are set on this config. If they are
   * not, throw a {@link VerifyError}. Intended to be called at the end of an initialization chain.
   *
   * @return this {@link SpindexerConfiguration} for chaining
   */
  public SpindexerConfiguration validate() {
    List<String> missing = new ArrayList<>();
    if (this.kTargetSpeedRPM == null) missing.add("kTargetSpeed");

    if (missing.size() != 0) {
      throw new VerifyError(
          "Config for " + kName + " does not have a set " + String.join(",", missing));
    }

    return this;
  }
}
