package com.aembot.frc2026.subsystems.turret;

import com.aembot.frc2026.config.subsystems.TalonFXTurretConfiguration;
import com.aembot.frc2026.state.subsystems.turret.TurretState;
import com.aembot.frc2026.subsystems.turret.io.TurretIO;
import com.aembot.lib.config.motors.MotorConfiguration;
import com.aembot.lib.core.encoders.CANCoderInputs;
import com.aembot.lib.core.motors.MotorInputs;
import com.aembot.lib.core.motors.interfaces.MotorIO;
import com.aembot.lib.core.tracing.Traced;
import com.aembot.lib.subsystems.base.MotorSubsystem;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.Logger;

/** Turret subsystm implementation */
public class TurretSubsystem
    extends MotorSubsystem<MotorInputs, MotorIO, MotorConfiguration<TalonFXConfiguration>> {
  private static final double TURRET_DEBUG_LOG_PERIOD_SECONDS = 0.1;

  /** IO to use for this subsystem */
  private final TurretIO io;

  /** Configuration for this turret */
  private final TalonFXTurretConfiguration config;

  /** TurretState instance to update */
  private final TurretState state;

  private final CANCoderInputs encoderAInputs = new CANCoderInputs();
  private final CANCoderInputs encoderBInputs = new CANCoderInputs();
  private final TurretInputs turretInputs = new TurretInputs();
  private double nextTurretDebugLogTimestampSeconds = 0.0;
  private final String turretEncoderALogKey;
  private final String turretEncoderBLogKey;
  private final String turretCalculatedRotLogKey;
  private final String turretLatencyPeriodicLogKey;
  private final String turretInputsLogKey;

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
    this.turretEncoderALogKey = logPrefixStandard + "/EncoderA";
    this.turretEncoderBLogKey = logPrefixStandard + "/EncoderB";
    this.turretCalculatedRotLogKey = logPrefixStandard + "/CalculatedTurretRot";
    this.turretLatencyPeriodicLogKey = logPrefixStandard + "/LatencyPeriodicMS";
    this.turretInputsLogKey = logPrefixInput + "/TurretInputs";

    io.getCANcoderA().updateInputs(encoderAInputs);
    io.getCANcoderB().updateInputs(encoderBInputs);
    // setPositionFromEncoders();

    setEncoderPosition(config.startingRotation);
  }

  private void setPositionFromEncoders() {
    double absolutePosition =
        config.getMechanismRotationsFromEncoders(
            io.getCANcoderA().getRawAngle() - 0.683838, io.getCANcoderB().getRawAngle() - 0.654541);

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
  @Traced
  public void periodic() {
    double timestamp = Timer.getFPGATimestamp();

    super.periodic();

    io.getCANcoderA().updateInputs(encoderAInputs);
    io.getCANcoderB().updateInputs(encoderBInputs);
    turretInputs.encoderAAbsolutePositionRotations = encoderAInputs.absolutePositionRotations;
    turretInputs.encoderAVelocityRotationsPerSecond = encoderAInputs.velocityRotationsPerSecond;
    turretInputs.encoderBAbsolutePositionRotations = encoderBInputs.absolutePositionRotations;
    turretInputs.encoderBVelocityRotationsPerSecond = encoderBInputs.velocityRotationsPerSecond;

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

    if (timestamp >= nextTurretDebugLogTimestampSeconds) {
      nextTurretDebugLogTimestampSeconds = timestamp + TURRET_DEBUG_LOG_PERIOD_SECONDS;
      Logger.recordOutput(turretEncoderALogKey, turretInputs.encoderAAbsolutePositionRotations);
      Logger.recordOutput(turretEncoderBLogKey, turretInputs.encoderBAbsolutePositionRotations);
      Logger.recordOutput(
          turretCalculatedRotLogKey,
          config.getMechanismRotationsFromEncoders(
              turretInputs.encoderAAbsolutePositionRotations - 0.683838,
              turretInputs.encoderBAbsolutePositionRotations - 0.654541));
    }

    state.updateTurretYaw(Rotation2d.fromDegrees(inputs.positionUnits));

    // Log latency with time between periodic being called and finishing
    Logger.recordOutput(turretLatencyPeriodicLogKey, (Timer.getFPGATimestamp() - timestamp) * 1000);
  }

  @Override
  @Traced
  public void updateLog(String standardPrefix, String inputPrefix) {
    Logger.processInputs(turretInputsLogKey, turretInputs);
    io.updateLog(standardPrefix, inputPrefix);
    super.updateLog(standardPrefix, inputPrefix);
  }
}
