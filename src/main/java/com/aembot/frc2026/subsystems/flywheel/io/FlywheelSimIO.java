package com.aembot.frc2026.subsystems.flywheel.io;

import com.aembot.lib.config.motors.MotorConfiguration;
import com.aembot.lib.config.subsystems.flywheel.simulation.SimulatedFlywheelConfiguration;
import com.aembot.lib.core.motors.interfaces.MotorIO;
import com.aembot.lib.subsystems.flywheel.simulation.SimulatedFlywheel;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

/** Flywheel IO implementation for simulation */
public class FlywheelSimIO extends FlywheelHardwareIO {
  private final SimulatedFlywheel simFlywheel;

  public FlywheelSimIO(
      MotorConfiguration<TalonFXConfiguration> flywheelConfig,
      SimulatedFlywheelConfiguration simulatedFlywheelConfig) {
    super(flywheelConfig);
    simFlywheel = new SimulatedFlywheel(flywheelConfig, simulatedFlywheelConfig);
  }

  @Override
  public void updateLog(String standardPrefix, String inputPrefix) {
    simFlywheel.updateLog(standardPrefix, inputPrefix);
  }

  @Override
  public MotorIO getLeadMotor() {
    return simFlywheel.getLeadTalon();
  }

  // @Override
  // public MotorIO[] getFollowerMotors() {
  //   return simFlywheel.getFollowerTalons();
  // }
}
