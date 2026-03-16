package com.aembot.lib.config.subsystems.intake.generic.run;

import com.aembot.lib.config.motors.MotorFollowersConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import java.util.ArrayList;
import java.util.List;

/** Config for subsystem with multiple motors that turns on and off with a preconfigured voltage. Works well for Texas Toast. */
public class BinaryVoltageMotorFollowerConfig {
  public final String kName;

  public MotorFollowersConfiguration<TalonFXConfiguration> kMotorConfigs;

  public Double kRunVoltage;

  public BinaryVoltageMotorFollowerConfig(String name) {
    this.kName = name;
  }

  /**
   * Sets the configs for the motors in this subsystem. Note that {@link #withMainSimConfig} must
   * also be called to set the simulated config for the main motor in order for the simulation to
   * work properly.
   *
   * @return This {@link BinaryVoltageMotorFollowerConfig} instance for chaining.
   */
  public BinaryVoltageMotorFollowerConfig withMotorConfigs(
      MotorFollowersConfiguration<TalonFXConfiguration> motorConfigs) {
    this.kMotorConfigs = motorConfigs;
    return this;
  }

  public BinaryVoltageMotorFollowerConfig withIntakeVoltage(double intakeVoltage) {
    this.kRunVoltage = intakeVoltage;
    return this;
  }

  /**
   * Check that all values required for a multi-motor intake roller subsystem are set on this
   * config. If they are not, throw a {@link VerifyError}. Intended to be called at the end of an
   * initialization chain.
   *
   * @return this {@link BinaryVoltageMotorFollowerConfig} for chaining
   */
  public BinaryVoltageMotorFollowerConfig validate() {
    List<String> missing = new ArrayList<>();
    if (this.kMotorConfigs == null) missing.add("kMotorConfigs");
    if (this.kRunVoltage == null) missing.add("kIntakeVoltage");

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
