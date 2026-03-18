package com.aembot.frc2026.subsystems.turret;

import com.aembot.frc2026.config.subsystems.TalonFXTurretConfiguration;
import com.aembot.frc2026.state.subsystems.turret.TurretState;
import com.aembot.frc2026.subsystems.turret.io.TurretIO;
import com.aembot.lib.config.motors.MotorConfiguration;
import com.aembot.lib.core.encoders.CANCoderInputs;
import com.aembot.lib.core.motors.MotorInputs;
import com.aembot.lib.core.motors.interfaces.MotorIO;
import com.aembot.lib.subsystems.base.MotorSubsystem;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
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

  /**
   * Compute absolute turret position using CRT with calibrated offsets.
   *
   * Offsets are applied here rather than via Phoenix6 MagnetOffset because calibration found
   * that encoder readings must be INVERTED (1.0 - raw) before the offset is added. Phoenix6's
   * MagnetOffset only adds an offset and cannot perform the inversion step.
   *
   * Calibration process invloved an exhaustive search compared CRT
   * output to motor encoder position (zeroed at startup). The search tested all combinations of
   * invert A/B (true/false), offset A (0-1 in 1/26 steps), offset B (0-1 in 1/34 steps), and gear
   * swap. The winning config (RMS error ~1.4 deg): invert both, offset A=21/26≈0.808, offset
   * B=5/34≈0.147. The offsets represent magnet misalignment; search resolution uses gear teeth
   * (13T/17T) as natural tick sizes.
   *
   * @return turret position in degrees, or -1 if CRT computation failed
   */
  private double getCalculatedTurretDeg() {
    // Invert readings then add calibrated offset - Phoenix6 MagnetOffset can't do inversion
    double adjustedA =
        MathUtil.inputModulus(
            (1.0 - encoderAInputs.absolutePositionRotations) + config.kCANcoderAOffset, 0, 1);
    double adjustedB =
        MathUtil.inputModulus(
            (1.0 - encoderBInputs.absolutePositionRotations) + config.kCANcoderBOffset, 0, 1);

    double crtRotations =
        config.getMechanismRotationsFromEncoders(
            adjustedA,
            adjustedB,
            config.kCANcoderAGearTeeth,
            config.kCANcoderBGearTeeth,
            config.kMechanismTeeth);

    if (crtRotations < 0) {
      return -1;
    }

    // CRT outputs absolute position directly - no starting rotation offset needed
    double degrees = (crtRotations % 1.0) * 360.0;
    return degrees < 0 ? degrees + 360.0 : degrees;
  }

  private void setPositionFromEncoders() {
    double calculatedDeg = getCalculatedTurretDeg();

    if (calculatedDeg < 0) {
      CommandScheduler.getInstance()
          .schedule(
              dutyCycleCommand(() -> 0)
                  .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                  .withName("DisableMotorCommand"));
      return;
    }

    // Zero encoder position based on CRT computation
    setEncoderPosition(calculatedDeg);
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

    Logger.recordOutput("calculatedTurretDeg", getCalculatedTurretDeg());

    state.updateTurretYaw(Rotation2d.fromDegrees(inputs.positionUnits));

    // Log latency with time between periodic being called and finishing
    Logger.recordOutput(
        logPrefixStandard + "/LatencyPeriodicMS", (Timer.getFPGATimestamp() - timestamp) * 1000);

    motorEnabled = SmartDashboard.getBoolean("Turret Enabled", motorEnabled);
  }

  @Override
  public void updateLog(String standardPrefix, String inputPrefix) {
    Logger.processInputs(inputPrefix, inputs);
    // Use unique prefixes for each CANcoder to prevent overwriting
    Logger.processInputs(inputPrefix + "/CANCoderA", encoderAInputs);
    Logger.processInputs(inputPrefix + "/CANCoderB", encoderBInputs);
    io.updateLog(standardPrefix, inputPrefix);
    super.updateLog(standardPrefix, inputPrefix);
  }
}
