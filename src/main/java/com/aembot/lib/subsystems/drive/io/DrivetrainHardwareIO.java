package com.aembot.lib.subsystems.drive.io;

import com.aembot.lib.config.subsystems.drive.DrivetrainConfiguration;
import com.aembot.lib.config.subsystems.drive.SwerveModuleConfiguration;
import com.aembot.lib.core.can.CANStatusLogger;
import com.aembot.lib.core.phoenix6.AEMSwerveDriveState;
import com.aembot.lib.core.tracing.Traced;
import com.aembot.lib.subsystems.aprilvision.util.AprilCameraOutput;
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
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;
import org.littletonrobotics.junction.Logger;

/** Hardware IO implementation of the drivetrain using the CTRE swerve API */
public class DrivetrainHardwareIO extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder>
    implements DrivetrainIO {
  /** Create a thread safe cached version of the telemetry that we can use to produce logs */
  private AtomicReference<AEMSwerveDriveState> swerveTelemetryCache = new AtomicReference<>();

  /**
   * Updates the odometry information from the drive train within our overall robot state as well as
   * updating the cache
   */
  protected Consumer<SwerveDriveState> swerveTelemetryConsumer =
      state -> {
        AEMSwerveDriveState aemState = AEMSwerveDriveState.fromSwerveDriveState(state);

        aemState.timestampRIOSynchronized =
            (Timer.getFPGATimestamp() - Utils.getCurrentTimeSeconds()) + aemState.Timestamp;

        swerveTelemetryCache.set(aemState);
      };

  private Matrix<N3, N1> stateStdDevs = null;

  private ArrayList<String> moduleNames = new ArrayList<>();
  private String[] moduleAbsoluteEncoderAngleLogKeys;
  private String[] moduleSteeringAngleLogKeys;
  private String[] moduleTargetSteeringAngleLogKeys;
  private String[] moduleDriveVelocityLogKeys;
  private String[] moduleTargetDriveVelocityLogKeys;
  private String cachedModuleLogPrefix = null;
  private String[] moduleAbsoluteEncoderAngleFullLogKeys;
  private String[] moduleSteeringAngleFullLogKeys;
  private String[] moduleTargetSteeringAngleFullLogKeys;
  private String[] moduleDriveVelocityFullLogKeys;
  private String[] moduleTargetDriveVelocityFullLogKeys;

  /* ----- Pigeon 2 Status Signals ----- */
  private final StatusSignal<AngularVelocity> angularPitchVelocity;
  private final StatusSignal<AngularVelocity> angularRollVelocity;
  private final StatusSignal<AngularVelocity> angularYawVelocity;

  private final StatusSignal<Angle> roll;
  private final StatusSignal<Angle> pitch;

  private final StatusSignal<LinearAcceleration> accelerationX;
  private final StatusSignal<LinearAcceleration> accelerationY;

  /* ----- CANCoder Absolute Position Signals ----- */
  @SuppressWarnings("unchecked")
  private final StatusSignal<Angle>[] absolutePositionSignals = new StatusSignal[4];

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
      moduleNames.add(i, swerveModuleConfigurations.get(i).moduleName);
      absolutePositionSignals[i] = getModule(i).getEncoder().getAbsolutePosition();
    }

    int moduleCount = swerveModuleConfigurations.size();
    moduleAbsoluteEncoderAngleLogKeys = new String[moduleCount];
    moduleSteeringAngleLogKeys = new String[moduleCount];
    moduleTargetSteeringAngleLogKeys = new String[moduleCount];
    moduleDriveVelocityLogKeys = new String[moduleCount];
    moduleTargetDriveVelocityLogKeys = new String[moduleCount];
    moduleAbsoluteEncoderAngleFullLogKeys = new String[moduleCount];
    moduleSteeringAngleFullLogKeys = new String[moduleCount];
    moduleTargetSteeringAngleFullLogKeys = new String[moduleCount];
    moduleDriveVelocityFullLogKeys = new String[moduleCount];
    moduleTargetDriveVelocityFullLogKeys = new String[moduleCount];
    for (int i = 0; i < moduleCount; i++) {
      String modulePrefix = "/Modules/" + moduleNames.get(i) + "/";
      moduleAbsoluteEncoderAngleLogKeys[i] = modulePrefix + "Absolute Encoder Angle";
      moduleSteeringAngleLogKeys[i] = modulePrefix + "Steering Angle";
      moduleTargetSteeringAngleLogKeys[i] = modulePrefix + "Target Steering Angle";
      moduleDriveVelocityLogKeys[i] = modulePrefix + "Drive Velocity";
      moduleTargetDriveVelocityLogKeys[i] = modulePrefix + "Target Drive Velocity";
    }

    // Set CANCoder signals to update at 100hz
    BaseStatusSignal.setUpdateFrequencyForAll(100, absolutePositionSignals);

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
  @Traced
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

    // Refresh and store absolute encoder positions
    BaseStatusSignal.refreshAll(absolutePositionSignals);
    for (int i = 0; i < absolutePositionSignals.length; i++) {
      inputs.absoluteEncoderPositions[i] = absolutePositionSignals[i].getValueAsDouble();
    }

    inputs.kinematics = getKinematics();
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
  @Traced
  public void resetOdometry(Pose2d pose) {
    super.resetPose(pose);
  }

  @Override
  @Traced
  public void logModules(DrivetrainInputs inputs, String prefix) {
    if (inputs.ModuleStates == null) return;
    initializeModuleLogKeysIfNeeded(prefix);
    for (int i = 0; i < getModules().length; i++) {
      Logger.recordOutput(
          moduleAbsoluteEncoderAngleFullLogKeys[i], inputs.absoluteEncoderPositions[i] * 360);
      Logger.recordOutput(moduleSteeringAngleFullLogKeys[i], inputs.ModuleStates[i].angle);
      Logger.recordOutput(moduleTargetSteeringAngleFullLogKeys[i], inputs.ModuleTargets[i].angle);
      Logger.recordOutput(
          moduleDriveVelocityFullLogKeys[i], inputs.ModuleStates[i].speedMetersPerSecond);
      Logger.recordOutput(
          moduleTargetDriveVelocityFullLogKeys[i], inputs.ModuleTargets[i].speedMetersPerSecond);
    }
  }

  private void initializeModuleLogKeysIfNeeded(String prefix) {
    if (prefix == null) prefix = "";
    if (prefix.equals(cachedModuleLogPrefix)) return;

    cachedModuleLogPrefix = prefix;
    for (int i = 0; i < moduleNames.size(); i++) {
      moduleAbsoluteEncoderAngleFullLogKeys[i] = prefix + moduleAbsoluteEncoderAngleLogKeys[i];
      moduleSteeringAngleFullLogKeys[i] = prefix + moduleSteeringAngleLogKeys[i];
      moduleTargetSteeringAngleFullLogKeys[i] = prefix + moduleTargetSteeringAngleLogKeys[i];
      moduleDriveVelocityFullLogKeys[i] = prefix + moduleDriveVelocityLogKeys[i];
      moduleTargetDriveVelocityFullLogKeys[i] = prefix + moduleTargetDriveVelocityLogKeys[i];
    }
  }

  @Override
  @Traced
  public void setRequest(SwerveRequest request) {
    super.setControl(request);
  }

  @Override
  @Traced
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

  @Override
  @Traced
  public void addVisionEstimation(AprilCameraOutput cameraOutput) {
    if (!Double.isNaN(cameraOutput.estimatedPose().stdDevs().xStdDev())
        && !Double.isNaN(cameraOutput.estimatedPose().latencyCompensatedPose().getX())) {
      var visPose = cameraOutput.estimatedPose().latencyCompensatedPose();
      addVisionMeasurement(
          new Pose2d(visPose.getX(), visPose.getY(), this.getRotation3d().toRotation2d()),
          Utils.fpgaToCurrentTime(cameraOutput.estimatedPose().timestampSeconds()),
          cameraOutput.estimatedPose().stdDevs().toMatrix());
    }
  }
}
