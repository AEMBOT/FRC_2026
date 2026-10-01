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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.DoubleSupplier;
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

  /* ---- FAST AIM ---- */

  /** How long without a drivetrain state before the per-loop aim takes over */
  private static final double FAST_AIM_STALE_SECONDS = 0.1;

  /** SmartDashboard toggle; when off the turret aims once per loop */
  private boolean fastAimAllowed = true;

  /** True while {@link #fastAimCommand} is running and fast aim is allowed */
  private volatile boolean fastAimEnabled = false;

  /** Copy of motorEnabled that the odometry thread can read safely */
  private volatile boolean fastAimMotorEnabled = true;

  private volatile double fastAimTargetDegrees = Double.NaN;
  private volatile double fastAimTimestampSeconds = 0.0;
  private volatile double fastAimRateDegPerSec = 0.0;

  /** Number of fast aim targets received since the last robot loop */
  private final AtomicInteger fastAimUpdates = new AtomicInteger();

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
    SmartDashboard.putBoolean("Turret Fast Aim Enabled", fastAimAllowed);
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

    AEMLogger.recordOutput(logPrefixStandard + "/FastAim/Enabled", fastAimEnabled);
    AEMLogger.recordOutput(logPrefixStandard + "/FastAim/TargetDegrees", fastAimTargetDegrees);
    AEMLogger.recordOutput(
        logPrefixStandard + "/FastAim/AgeMs",
        (Timer.getFPGATimestamp() - fastAimTimestampSeconds) * 1000);
    AEMLogger.recordOutput(
        logPrefixStandard + "/FastAim/UpdatesSinceLastLoop", fastAimUpdates.getAndSet(0));
    AEMLogger.recordOutput(logPrefixStandard + "/FastAim/RateDegPerSec", fastAimRateDegPerSec);

    // Log latency with time between periodic being called and finishing
    AEMLogger.recordOutput(
        logPrefixStandard + "/LatencyPeriodicMS", (Timer.getFPGATimestamp() - timestamp) * 1000);

    motorEnabled = SmartDashboard.getBoolean("Turret Enabled", motorEnabled);
    fastAimMotorEnabled = motorEnabled;
    fastAimAllowed = SmartDashboard.getBoolean("Turret Fast Aim Enabled", fastAimAllowed);
  }

  /**
   * Accept a turret target computed from the latest drivetrain state. Runs on the drivetrain
   * odometry thread at 250 Hz, so it must not log; the values are logged from {@link #periodic}. It
   * commands the motor directly because setSmartPositionSetpointImpl logs.
   *
   * @param targetDegrees Turret target in degrees
   * @param timestampSeconds FPGA timestamp of the drivetrain state the target was computed from
   */
  public void acceptFastAim(double targetDegrees, double timestampSeconds) {
    double previousTargetDegrees = fastAimTargetDegrees;
    double dtSeconds = timestampSeconds - fastAimTimestampSeconds;
    if (!Double.isNaN(previousTargetDegrees) && dtSeconds > 0) {
      fastAimRateDegPerSec =
          MathUtil.inputModulus(targetDegrees - previousTargetDegrees, -180, 180) / dtSeconds;
    }

    fastAimTargetDegrees = targetDegrees;
    fastAimTimestampSeconds = timestampSeconds;
    fastAimUpdates.incrementAndGet();

    if (fastAimEnabled && fastAimMotorEnabled) {
      io.getMotor().setSmartPositionSetpoint(targetDegrees, 0);
    }
  }

  /**
   * Aim the turret from the drivetrain odometry thread through {@link #acceptFastAim}. Every loop
   * this still evaluates the per-loop target, which keeps the aim's cached state fresh, and sends
   * it itself when fast aim can't: no drivetrain state for {@value #FAST_AIM_STALE_SECONDS} s
   * (replay, or no listener registered), fast aim toggled off, or the turret disabled (the per-loop
   * path then holds the motor at 0 V).
   *
   * @param perLoopTargetDegrees Supplier of the turret target computed once per loop
   * @return The command; any other command requiring the turret ends it and takes the motor
   */
  public Command fastAimCommand(DoubleSupplier perLoopTargetDegrees) {
    return runEnd(
            () -> {
              double perLoopTarget = perLoopTargetDegrees.getAsDouble();
              fastAimEnabled = fastAimAllowed;

              boolean fastAimStale =
                  Timer.getFPGATimestamp() - fastAimTimestampSeconds > FAST_AIM_STALE_SECONDS;
              if (!fastAimEnabled || !motorEnabled || fastAimStale) {
                setSmartPositionSetpointImpl(perLoopTarget);
              }
            },
            () -> {
              fastAimEnabled = false;
            })
        .withName("FastAimTowardsGoal");
  }

  @Override
  public void updateLog(String standardPrefix, String inputPrefix) {
    Logger.processInputs(inputPrefix, inputs);
    Logger.processInputs(inputPrefix, encoderAInputs);
    Logger.processInputs(inputPrefix, encoderBInputs);
    io.updateLog(standardPrefix, inputPrefix);
    super.updateLog(standardPrefix, inputPrefix);
  }
}
