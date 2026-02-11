package com.aembot.frc2026.subsystems;

import com.aembot.frc2026.constants.RobotRuntimeConstants;
import com.aembot.frc2026.subsystems.flywheel.FlywheelSubsystem;
import com.aembot.frc2026.subsystems.flywheel.io.FlywheelHardwareIO;
import com.aembot.frc2026.subsystems.flywheel.io.FlywheelSimIO;
import com.aembot.lib.config.motors.MotorFollowersConfiguration;
import com.aembot.lib.config.subsystems.flywheel.simulation.SimulatedFlywheelConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

/** Subsystem factories intended to be called from RobotContainer */
public class SubsystemFactory {
  public static FlywheelSubsystem createFlywheelSubsystem() {
    FlywheelSubsystem flywheelSubsystem = null;
    MotorFollowersConfiguration<TalonFXConfiguration> flywheelMotorsConfig =
        RobotRuntimeConstants.CURRENT_ROBOT_CONFIGS.getFlywheelConfiguration();
    SimulatedFlywheelConfiguration flywheelSimConfig =
        RobotRuntimeConstants.CURRENT_ROBOT_CONFIGS.getSimulatedFlywheelConfiguration();

    switch (RobotRuntimeConstants.CURRENT_RUNTIME_MODE) {
      case SIM:
        FlywheelSimIO flywheelIO = new FlywheelSimIO(flywheelMotorsConfig, flywheelSimConfig);
        flywheelSubsystem = new FlywheelSubsystem(flywheelMotorsConfig, flywheelIO);
        break;
      // go to default
      case REPLAY:
      case REAL:
      default:
        flywheelSubsystem =
            new FlywheelSubsystem(
                flywheelMotorsConfig, new FlywheelHardwareIO(flywheelMotorsConfig));
        break;
    }
    return flywheelSubsystem;
  }
}
