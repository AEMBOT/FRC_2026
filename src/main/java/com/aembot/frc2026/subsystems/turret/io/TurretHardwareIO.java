// copy pasted modified from elevatorhardwareio.java
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.aembot.frc2026.subsystems.turret.io;

// specify type here and in sim make a new one. cancodersimio. cancoderhardwareio.

import com.aembot.lib.config.motors.MotorConfiguration;
import com.aembot.lib.core.motors.interfaces.MotorIO;
import com.aembot.lib.core.motors.io.MotorIOTalonFX;
import com.aembot.lib.subsystems.base.turret.TurretIO;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

// import com.aembot.lib.core.motors.factories.TalonFXFactory;

/** Hardware IO implementation for a turret */
public class TurretHardwareIO implements TurretIO {
  private final MotorIOTalonFX singleMotor;

  // private final MotorIOTalonFX[] followerMotors;

  public TurretHardwareIO(MotorConfiguration<TalonFXConfiguration> turretConfig) {
    singleMotor = new MotorIOTalonFX(turretConfig);
    // leadMotor = TalonFXFactory.createIO(turretConfig);

    // Setup our follower motors based on the configuration
    // followerMotors = new MotorIOTalonFX[] {
    //     new MotorIOTalonFX(turretConfig.followerConfigurations.get(0).config)
    // };
  }

  @Override
  public void updateLog(String standardPrefix, String inputPrefix) {}

  @Override
  public MotorIO getSingleMotor() {
    return singleMotor;
  }

  // @Override
  // public MotorIO[] getFollowerMotor() {
  //     return followerMotors;
  // }

}
