package com.aembot.frc2026.subsystems.flywheel.io;

import com.aembot.lib.config.motors.MotorFollowersConfiguration;
import com.aembot.lib.core.motors.factories.TalonFXFactory;
import com.aembot.lib.core.motors.interfaces.MotorIO;
import com.aembot.lib.core.motors.io.MotorIOTalonFX;
import com.aembot.lib.subsystems.flywheel.FlywheelIO;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

/** Hardware IO implementation for a flywheel */
public class FlywheelHardwareIO implements FlywheelIO {
  private final MotorIOTalonFX leadMotor;
  private final MotorIOTalonFX[] followerMotors;

  public FlywheelHardwareIO(MotorFollowersConfiguration<TalonFXConfiguration> flywheelConfig) {
    // Setup our leader motor based on the configuration
    leadMotor = TalonFXFactory.createIO(flywheelConfig);

    // Setup our follower motors based on the configuration
    followerMotors =
        new MotorIOTalonFX[] {
          TalonFXFactory.createIO(flywheelConfig.followerConfigurations.get(0).config)
        };
  }

  @Override
  public void updateLog(String standardPrefix, String inputPrefix) {}

  @Override
  public MotorIO getLeadMotor() {
    return leadMotor;
  }

  @Override
  public MotorIO[] getFollowerMotors() {
    return followerMotors;
  }
}
