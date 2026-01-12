package com.aembot.lib.config.motors.factories;

import com.aembot.lib.config.motors.MotorConfiguration;
import com.aembot.lib.config.motors.MotorFollowersConfiguration;
import com.aembot.lib.core.can.CANDeviceID;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class TalonFXFactoryConfiguration {
  public static MotorFollowersConfiguration.FollowerConfiguration<TalonFXConfiguration>
      generateFollowerTalonFXConfiguration() {
    return new MotorFollowersConfiguration.FollowerConfiguration<>(
        new MotorConfiguration<TalonFXConfiguration>().withConfig(new TalonFXConfiguration()));
  }

  public static MotorFollowersConfiguration.FollowerConfiguration<TalonFXConfiguration>
      generateFollowerTalonFXConfiguration(String name, CANDeviceID device) {
    return new MotorFollowersConfiguration.FollowerConfiguration<>(
        new MotorConfiguration<TalonFXConfiguration>()
            .withConfig(new TalonFXConfiguration())
            .withName(name)
            .withCANDevice(device));
  }
}
