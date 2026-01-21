package com.aembot.lib.subsystems.drive.io;

import com.aembot.frc2026.state.RobotStateYearly;
import com.aembot.lib.config.subsystems.drive.DrivetrainConfiguration;
import com.aembot.lib.config.subsystems.drive.SwerveModuleConfiguration;
import com.aembot.lib.core.can.CANStatusLogger;
import com.aembot.lib.subsystems.drive.DrivetrainInputs;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** Hardware IO implementation of the drivetrain using the CTRE swerve API */
public class DrivetrainHardwareIO extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder>
    implements DrivetrainIO {
  /** Create a thread safe cached version of the telemetry that we can use to produce logs */
  private AtomicReference<SwerveDriveState> swerveTelemetryCache = new AtomicReference<>();

  /**
   * Updates the odometry information from the drive train within our overall robot state as well as
   * updating the cache
   */
  protected Consumer<SwerveDriveState> swerveTelemetryConsumer =
      state -> {
        // Update the state via deep-copy
        swerveTelemetryCache.set(state.clone());

        RobotStateYearly.get()
            .addOdometryMeasurement(
                // Synchronize the RoboRIO clock with the system time and return the rio time of the
                // state
                (Timer.getFPGATimestamp() - Utils.getCurrentTimeSeconds()) + state.Timestamp,

                // New pose of the swerve drive
                state.Pose);
      };

  private Matrix<N3, N1> stateStdDevs = null;

  private ArrayList<String> moduleNames;

  /* ----- Pigeon 2 Status Signals ----- */
  private final StatusSignal<AngularVelocity> angularPitchVelocity;
  private final StatusSignal<AngularVelocity> angularRollVelocity;
  private final StatusSignal<AngularVelocity> angularYawVelocity;

  private final StatusSignal<Angle> roll;
  private final StatusSignal<Angle> pitch;

  private final StatusSignal<LinearAcceleration> accelerationX;
  private final StatusSignal<LinearAcceleration> accelerationY;

  /**
   * Construct the IO layer for a real drivetrain
   *
   * @param driveTrainConfiguration Config for the full drivetrain
   * @param swerveModuleConfigurations Config for each individual swerve module
   */
  public DrivetrainHardwareIO(
      DrivetrainConfiguration driveTrainConfiguration,
      List<
              SwerveModuleConfiguration<
                  TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>>
          swerveModuleConfigurations) {
    // Create the CTRE swerve drive train from our robot configuration
    super(
        TalonFX::new,
        TalonFX::new,
        CANcoder::new,
        driveTrainConfiguration.ctreDriveConstants,
        250.0,
        driveTrainConfiguration.ctreModuleConstants);

    // Retrieve all Pigeon2 signals
    angularPitchVelocity = getPigeon2().getAngularVelocityYWorld();
    angularRollVelocity = getPigeon2().getAngularVelocityXWorld();
    angularYawVelocity = getPigeon2().getAngularVelocityZWorld();
    roll = getPigeon2().getRoll();
    pitch = getPigeon2().getPitch();
    accelerationX = getPigeon2().getAccelerationX();
    accelerationY = getPigeon2().getAccelerationY();

    for (int i = 0; i < swerveModuleConfigurations.size(); i++) {
      moduleNames.set(i, swerveModuleConfigurations.get(i).moduleName);
    }

    // Set yaw velocity to update at 250 hz; we care more about this value
    BaseStatusSignal.setUpdateFrequencyForAll(250, angularYawVelocity);

    // Set rest to update at 100; we don't care about them (much) >:c
    BaseStatusSignal.setUpdateFrequencyForAll(
        100, angularPitchVelocity, angularRollVelocity, roll, pitch, accelerationX, accelerationY);

    CANStatusLogger.get(driveTrainConfiguration.ctreDriveConstants.CANBusName)
        .registerSwerveDrivetrain(
            this, swerveModuleConfigurations, driveTrainConfiguration.gyroDeviceID);

    // Highest priority
    this.getOdometryThread().setThreadPriority(99);

    // Register the telemetry consumer with the underlying CTRE swerve drive train
    registerTelemetry(swerveTelemetryConsumer);
  }

  @Override
  public void updateInputs(DrivetrainInputs inputs) {
    // return and we will try again next loop. this could mean that CAN is not running and we are in
    // replay mode
    if (swerveTelemetryCache.get() == null) return;

    inputs.importSwerveDriveState(swerveTelemetryCache.get());

    // Update all gyro signals
    BaseStatusSignal.refreshAll(
        angularRollVelocity,
        angularPitchVelocity,
        angularYawVelocity,
        pitch,
        roll,
        accelerationX,
        accelerationY);

    inputs.gyroYawAngle = inputs.Pose.getRotation().getDegrees();
    inputs.yawAngularVelocity = angularYawVelocity.getValueAsDouble();
    inputs.rollAngularVelocity = angularRollVelocity.getValueAsDouble();
    inputs.pitchAngularVelocity = angularPitchVelocity.getValueAsDouble();
    inputs.pitch = pitch.getValueAsDouble();
    inputs.roll = roll.getValueAsDouble();
    inputs.accelX = accelerationX.getValueAsDouble();
    inputs.accelY = accelerationY.getValueAsDouble();
  }

  @Override
  public void resetOdometry(Pose2d pose) {
    super.resetPose(pose);
  }

  @Override
  public void logModules(SwerveDriveState state, String prefix) {
    if (state.ModuleStates == null) return;
    final String modulePrefix = prefix + "/Modules/";
    for (int i = 0; i < getModules().length; i++) {
      Logger.recordOutput(
          modulePrefix + moduleNames.get(i) + "/Absolute Encoder Angle",
          getModule(i).getEncoder().getAbsolutePosition().getValueAsDouble() * 360);
      Logger.recordOutput(
          modulePrefix + moduleNames.get(i) + "/Steering Angle", state.ModuleStates[i].angle);
      Logger.recordOutput(
          modulePrefix + moduleNames.get(i) + "/Target Steering Angle",
          state.ModuleTargets[i].angle);
      Logger.recordOutput(
          modulePrefix + moduleNames.get(i) + "/Drive Velocity",
          state.ModuleStates[i].speedMetersPerSecond);
      Logger.recordOutput(
          modulePrefix + moduleNames.get(i) + "/Target Drive Velocity",
          state.ModuleTargets[i].speedMetersPerSecond);
    }
  }

  @Override
  public SwerveDriveKinematics getSwerveKinematics() {
    return getKinematics();
  }

  @Override
  public void setRequest(SwerveRequest request) {
    super.setControl(request);
  }

  @Override
  public Command continuousRequestCommand(
      Supplier<SwerveRequest> requestSupplier, Subsystem... subsystemsRequired) {
    return Commands.run(() -> this.setControl(requestSupplier.get()), subsystemsRequired);
  }

  @Override
  public void setOdometryStdDevs(double xStd, double yStd, double rotStd) {
    // Initialize only once so we don't need to run garbage collection on it
    if (stateStdDevs == null) {
      stateStdDevs = VecBuilder.fill(xStd, yStd, rotStd);
    } else {
      stateStdDevs.set(0, 0, xStd);
      stateStdDevs.set(1, 0, yStd);
      stateStdDevs.set(2, 0, rotStd);
    }

    this.setStateStdDevs(stateStdDevs);
  }
}
