package com.aembot.frc2026.config.robots;

import com.aembot.frc2026.config.RobotConfiguration;
import com.aembot.lib.config.robot.PhysicalConfiguration;
import com.aembot.lib.config.subsystems.drive.DrivetrainConfiguration;
import com.aembot.lib.config.subsystems.drive.SwerveModuleConfiguration;
import com.aembot.lib.config.subsystems.drive.simulation.DrivetrainSimConfiguration;
import com.aembot.lib.config.subsystems.vision.CameraConfiguration;
import com.aembot.lib.config.subsystems.vision.SimulatedCameraConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.util.Units;
import java.util.List;

// TODO pretty much everything here is a placeholder
/** {@link RobotConfiguration} for the robot Timothy */
public class ProductionConfig extends RobotConfiguration {
  private static final String ROBOT_NAME = "Timothy";

  private static final String DRIVETRAIN_BUS_NAME = "drivetrain";
  private static final List<String> CAN_BUS_NAMES = List.of(DRIVETRAIN_BUS_NAME);

  private static final PhysicalConfiguration PHYSICAL_CONFIGURATION =
      new PhysicalConfiguration()
          .withRobotWeightPounds(150)
          .withWheelBaseLengthM(Units.inchesToMeters(22.75))
          .withWheelTrackWidthM(Units.inchesToMeters(22.75))
          .withBumperLengthM(Units.inchesToMeters(35.625))
          .withBumperWidthM(Units.inchesToMeters(35.625))
          .withWheelCoefficientOfFriction(1.2);

  private static final ProductionDrivetrainConfig DRIVETRAIN_CONFIG =
      new ProductionDrivetrainConfig(PHYSICAL_CONFIGURATION, DRIVETRAIN_BUS_NAME);

  private static final ProductionCameraConfig CAMERA_CONFIG = new ProductionCameraConfig();

  @Override
  public String getRobotName() {
    return ROBOT_NAME;
  }

  @Override
  public List<String> getCANBusNames() {
    return CAN_BUS_NAMES;
  }

  @Override
  public DrivetrainConfiguration getDrivetrainConfiguration() {
    return DRIVETRAIN_CONFIG.drivetrainConfiguration;
  }

  @Override
  public List<
          SwerveModuleConfiguration<
              TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>>
      getSwerveConfigurations() {
    return DRIVETRAIN_CONFIG.moduleConfigs.configurations;
  }

  @Override
  public DrivetrainSimConfiguration getSimulatedDrivetrainConfiguration() {
    return DRIVETRAIN_CONFIG.simulatedDrivetrainConfiguration;
  }

  @Override
  public List<CameraConfiguration> getCameraConfigurations() {
    return CAMERA_CONFIG.cameraConfigurations;
  }

  @Override
  public List<SimulatedCameraConfiguration> getSimulatedCameraConfigurations() {
    return CAMERA_CONFIG.simConfigurations;
  }
}
