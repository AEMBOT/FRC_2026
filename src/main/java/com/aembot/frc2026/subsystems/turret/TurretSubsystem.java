package com.aembot.frc2026.subsystems.turret;

import com.aembot.frc2026.config.subsystems.TalonFXTurretConfiguration;
import com.aembot.frc2026.state.subsystems.turret.TurretState;
import com.aembot.frc2026.subsystems.turret.io.TurretIO;
import com.aembot.lib.config.motors.MotorConfiguration;
import com.aembot.lib.core.encoders.CANCoderInputs;
import com.aembot.lib.core.logging.AEMLogger;
import com.aembot.lib.core.motors.MotorInputs;
import com.aembot.lib.core.motors.interfaces.MotorIO;
import com.aembot.lib.subsystems.base.MotorSubsystem;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.Logger;

/** Turret subsystm implementation */
public class TurretSubsystem
    extends MotorSubsystem<MotorInputs, MotorIO, MotorConfiguration<TalonFXConfiguration>> {

  /** IO to use for this subsystem */
  private final TurretIO io;

  /** Configuration for this turret */
  private final TalonFXTurretConfiguration config;

  /** TurretState instance to update */
  private final TurretState state;

  private final CANCoderInputs encoderAInputs = new CANCoderInputs();
  private final CANCoderInputs encoderBInputs = new CANCoderInputs();

  private final TurretInputs kTrackingInputs = new TurretInputs();
  private final String kTrackingInputsKey = logPrefixInput + "/Tracking";
  private double lastRequestTimestampSeconds = Double.NaN;
  private double lastLegalTargetDegrees = Double.NaN;
  private double lastReadTimestampSeconds = Double.NaN;
  private final String kRawTargetDegreesKey = logPrefixStandard + "/Tracking/RawTargetDegrees";
  private final String kLegalTargetDegreesKey = logPrefixStandard + "/Tracking/LegalTargetDegrees";
  private final String kRequestTimestampSecondsKey =
      logPrefixStandard + "/Tracking/RequestTimestampSeconds";
  private final String kRequestIntervalSecondsKey =
      logPrefixStandard + "/Tracking/RequestIntervalSeconds";
  private final String kRequestMotorEnabledKey =
      logPrefixStandard + "/Tracking/RequestMotorEnabled";
  private final String kRequestSlotKey = logPrefixStandard + "/Tracking/RequestSlot";
  private final String kSnapshotLegalTargetDegreesKey =
      logPrefixStandard + "/Tracking/SnapshotLegalTargetDegrees";
  private final String kSnapshotRequestTimestampSecondsKey =
      logPrefixStandard + "/Tracking/SnapshotRequestTimestampSeconds";
  private final String kSnapshotRequestAgeSecondsKey =
      logPrefixStandard + "/Tracking/SnapshotRequestAgeSeconds";
  private final String kLoopIntervalSecondsKey =
      logPrefixStandard + "/Tracking/LoopIntervalSeconds";
  private final String kDriverStationEnabledKey =
      logPrefixStandard + "/Tracking/DriverStationEnabled";
  private final String kMotorEnabledKey = logPrefixStandard + "/Tracking/MotorEnabled";

  /**
   * Create a new turret subsystem
   *
   * @param config Configuration for this subsystem
   * @param io IO to use
   */
  public TurretSubsystem(
      TalonFXTurretConfiguration config, TurretIO io, TurretState turretStateInstance) {

    super(config.kName, new MotorInputs(), io.getMotor(), config.kRealMotorConfig);

    this.io = io;
    this.config = config;
    this.state = turretStateInstance;

    io.getCANcoderA().updateInputs(encoderAInputs);
    io.getCANcoderB().updateInputs(encoderBInputs);
    // setPositionFromEncoders();

    setEncoderPosition(config.startingRotation);

    SmartDashboard.putBoolean("Turret Enabled", motorEnabled);
  }

  private void setPositionFromEncoders() {
    double absolutePosition =
        config.getMechanismRotationsFromEncoders(
            MathUtil.inputModulus(io.getCANcoderA().getRawAngle(), 0, 1),
            MathUtil.inputModulus(io.getCANcoderB().getRawAngle(), 0, 1),
            config.kCANcoderAGearTeeth,
            config.kCANcoderBGearTeeth,
            config.kMechanismTeeth);

    if (absolutePosition == -1) {
      CommandScheduler.getInstance()
          .schedule(
              dutyCycleCommand(() -> 0)
                  .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                  .withName("DisableMotorCommand"));
    }

    // Zero encoder position based on cancoders
    setEncoderPosition(absolutePosition);
  }

  @Override
  public void periodic() {
    double timestamp = Timer.getFPGATimestamp();

    super.periodic();

    io.getCANcoderA().updateInputs(encoderAInputs);
    io.getCANcoderB().updateInputs(encoderBInputs);

    // If motor has reset (e.g. brownout) then rezero
    if (io.getMotor().hasResetOccurred()) {
      // setPositionFromEncoders();
      // We're kinda screwed so just disable turret
      CommandScheduler.getInstance()
          .schedule(
              dutyCycleCommand(() -> 0)
                  .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                  .withName("DisableMotorCommand"));
    }

    // setPositionFromEncoders();

    AEMLogger.recordOutput("encoderA", io.getCANcoderA().getRawAngle());
    AEMLogger.recordOutput("encoderB", io.getCANcoderB().getRawAngle());

    AEMLogger.recordOutput(
        "calculatedTurretRot",
        config.getMechanismRotationsFromEncoders(
            encoderAInputs.absolutePositionRotations,
            encoderBInputs.absolutePositionRotations,
            config.kCANcoderAGearTeeth,
            config.kCANcoderBGearTeeth,
            config.kMechanismTeeth));

    state.updateTurretYaw(Rotation2d.fromDegrees(inputs.positionUnits));

    // Log latency with time between periodic being called and finishing
    AEMLogger.recordOutput(
        logPrefixStandard + "/LatencyPeriodicMS", (Timer.getFPGATimestamp() - timestamp) * 1000);

    motorEnabled = SmartDashboard.getBoolean("Turret Enabled", motorEnabled);
  }

  @Override
  protected void setSmartPositionSetpointImpl(double position, int slot) {
    double now = Timer.getFPGATimestamp();
    double legal =
        MathUtil.clamp(position, motorConfig.kMinPositionUnits, motorConfig.kMaxPositionUnits);
    Logger.recordOutput(kRawTargetDegreesKey, position);
    Logger.recordOutput(kLegalTargetDegreesKey, legal);
    Logger.recordOutput(kRequestTimestampSecondsKey, now);
    Logger.recordOutput(kRequestIntervalSecondsKey, now - lastRequestTimestampSeconds);
    Logger.recordOutput(kRequestMotorEnabledKey, motorEnabled);
    Logger.recordOutput(kRequestSlotKey, (double) slot);
    lastRequestTimestampSeconds = now;
    lastLegalTargetDegrees = legal;
    super.setSmartPositionSetpointImpl(position, slot);
  }

  @Override
  public void updateLog(String standardPrefix, String inputPrefix) {
    // MotorSubsystem has just read motor feedback. Read the additional profile signals
    // before commands execute, and identify the previously issued request beside this snapshot.
    io.updateInputs(kTrackingInputs);
    Logger.processInputs(kTrackingInputsKey, kTrackingInputs);
    double now = Timer.getFPGATimestamp();
    Logger.recordOutput(kSnapshotLegalTargetDegreesKey, lastLegalTargetDegrees);
    Logger.recordOutput(kSnapshotRequestTimestampSecondsKey, lastRequestTimestampSeconds);
    Logger.recordOutput(kSnapshotRequestAgeSecondsKey, now - lastRequestTimestampSeconds);
    Logger.recordOutput(kLoopIntervalSecondsKey, now - lastReadTimestampSeconds);
    Logger.recordOutput(kDriverStationEnabledKey, DriverStation.isEnabled());
    Logger.recordOutput(kMotorEnabledKey, motorEnabled);
    lastReadTimestampSeconds = now;

    Logger.processInputs(inputPrefix, inputs);
    Logger.processInputs(inputPrefix, encoderAInputs);
    Logger.processInputs(inputPrefix, encoderBInputs);
    io.updateLog(standardPrefix, inputPrefix);
    super.updateLog(standardPrefix, inputPrefix);
  }
}
