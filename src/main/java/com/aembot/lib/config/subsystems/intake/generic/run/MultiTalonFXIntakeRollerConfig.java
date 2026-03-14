package com.aembot.lib.config.subsystems.intake.generic.run;

import com.aembot.lib.config.motors.MotorFollowersConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import java.util.ArrayList;
import java.util.List;

/** Config for a intake roller subsystem with multiple motors. Works well for Texas Toast. */
public class MultiTalonFXIntakeRollerConfig {
  public final String kName;

  public MotorFollowersConfiguration<TalonFXConfiguration> kMotorConfigs;

  public Double kIntakeVoltage;

  public MultiTalonFXIntakeRollerConfig(String name) {
    this.kName = name;
  }

  /**
   * Sets the configs for the motors in this subsystem. Note that {@link #withMainSimConfig} must
   * also be called to set the simulated config for the main motor in order for the simulation to
   * work properly.
   *
   * @return This {@link MultiTalonFXIntakeRollerConfig} instance for chaining.
   */
  public MultiTalonFXIntakeRollerConfig withMotorConfigs(
      MotorFollowersConfiguration<TalonFXConfiguration> motorConfigs) {
    this.kMotorConfigs = motorConfigs;
    return this;
  }

  public MultiTalonFXIntakeRollerConfig withIntakeVoltage(double intakeVoltage) {
    this.kIntakeVoltage = intakeVoltage;
    return this;
  }

  /**
   * Check that all values required for a multi-motor intake roller subsystem are set on this
   * config. If they are not, throw a {@link VerifyError}. Intended to be called at the end of an
   * initialization chain.
   *
   * @return this {@link MultiTalonFXIntakeRollerConfig} for chaining
   */
  public MultiTalonFXIntakeRollerConfig validate() {
    List<String> missing = new ArrayList<>();
    if (this.kMotorConfigs == null) missing.add("kMotorConfigs");
    if (this.kIntakeVoltage == null) missing.add("kIntakeVoltage");

    String lowerErrors = "";
    if (this.kMotorConfigs != null) {
      try {
        this.kMotorConfigs.validate();
      } catch (VerifyError e) {
        lowerErrors += e.getMessage() + "\n";
      }
    }

    if (missing.size() != 0 || !lowerErrors.isEmpty()) {
      throw new VerifyError(
          "Config for "
              + kName
              + " does not have a set "
              + String.join(",", missing)
              + "\n"
              + lowerErrors);
    }

    return this;
  }
}
