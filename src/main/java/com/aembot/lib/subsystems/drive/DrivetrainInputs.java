package com.aembot.lib.subsystems.drive;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class DrivetrainInputs extends SwerveDriveState implements LoggableInputs {
  public SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(
          new Translation2d[] {
            new Translation2d(), new Translation2d(), new Translation2d(), new Translation2d()
          });

  public double gyroYawAngle = 0.0;

  public double yawAngularVelocity = 0.0;
  public double rollAngularVelocity = 0.0;
  public double pitchAngularVelocity = 0.0;

  public double pitch = 0.0;
  public double roll = 0.0;

  public double accelX = 0.0;
  public double accelY = 0.0;

  public DrivetrainInputs() {
    this.Pose = Pose2d.kZero;
  }

  /**
   * Load the state from a SwerveDriveState
   *
   * @param stateIn The state we are loading from
   */
  public void importSwerveDriveState(SwerveDriveState stateIn) {
    this.Pose = stateIn.Pose;
    this.SuccessfulDaqs = stateIn.SuccessfulDaqs;
    this.FailedDaqs = stateIn.FailedDaqs;
    this.ModuleStates = stateIn.ModuleStates;
    this.ModuleTargets = stateIn.ModuleTargets;
    this.Speeds = stateIn.Speeds;
    this.OdometryPeriod = stateIn.OdometryPeriod;
  }

  @Override
  public void toLog(LogTable table) {
    table.put("SwerveDriveKinematics", kinematics);

    table.put("RobotYawAngle", gyroYawAngle);
    table.put("Pose", Pose);
    table.put("Speeds", Speeds);
    table.put("OdometryPeriod", OdometryPeriod);
    table.put("Timestamp", Timestamp);

    // Log Module States (Struct array support is best, manual fallback shown)
    table.put("ModuleStates", ModuleStates);
    table.put("ModuleTargets", ModuleTargets);

    table.put("YawAngularVelocity", yawAngularVelocity);
    table.put("RollAngularVelocity", rollAngularVelocity);
    table.put("PitchAngularVelocity", pitchAngularVelocity);

    table.put("Pitch", pitch);
    table.put("Roll", roll);

    table.put("AccelerationX", accelX);
    table.put("AccelerationY", accelY);
  }

  @Override
  public void fromLog(LogTable table) {
    kinematics = table.get("SwerveDriveKinematics", kinematics);

    gyroYawAngle = table.get("RobotYawAngle", gyroYawAngle);
    Pose = table.get("Pose", Pose);
    Speeds = table.get("Speeds", Speeds);
    OdometryPeriod = table.get("OdometryPeriod", OdometryPeriod);
    Timestamp = table.get("Timestamp", Timestamp);

    ModuleStates = table.get("ModuleStates", new SwerveModuleState[4]);
    ModuleTargets = table.get("ModuleTargets", new SwerveModuleState[4]);

    yawAngularVelocity = table.get("YawAngularVelocity", yawAngularVelocity);
    rollAngularVelocity = table.get("RollAngularVelocity", rollAngularVelocity);
    pitchAngularVelocity = table.get("PitchAngularVelocity", pitchAngularVelocity);

    pitch = table.get("Pitch", pitch);
    roll = table.get("Roll", roll);
    accelX = table.get("AccelerationX", accelX);
    accelY = table.get("AccelerationY", accelY);
  }
}
