package com.aembot.frc2026.subsystems.turret;

import com.aembot.frc2026.config.subsystems.TalonFXTurretConfiguration;
import com.aembot.frc2026.subsystems.turret.io.TurretIO;
import com.aembot.lib.config.motors.MotorConfiguration;
import com.aembot.lib.core.motors.MotorInputs;
import com.aembot.lib.core.motors.interfaces.MotorIO;
import com.aembot.lib.subsystems.base.MotorSubsystem;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.Logger;

/** Turret subsystm implementation */
public class TurretSubsystem
    extends MotorSubsystem<MotorInputs, MotorIO, MotorConfiguration<TalonFXConfiguration>> {

  /** IO to use for this subsystem */
  public final TurretIO io;

  /** Configuration for this turret */
  public final TalonFXTurretConfiguration config;

  /**
   * Create a new turret subsystem
   *
   * @param config Configuration for this subsystem
   * @param io IO to use
   */
  public TurretSubsystem(TalonFXTurretConfiguration config, TurretIO io) {

    super(config.kName, new MotorInputs(), io.getMotor(), config.kRealMotorConfig);

    this.io = io;
    this.config = config;

    setPositionFromEncoders();
  }

  private void setPositionFromEncoders() {
    double absolutePosition =
        config.getMechanismRotationsFromEncoders(
            io.getCANcoderA().getRawAngle(), io.getCANcoderB().getRawAngle());

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
    super.periodic();
    // If motor has reset (e.g. brownout) then rezero
    if (io.getMotor().hasResetOccurred()) {
      setPositionFromEncoders();
    }
  }

  @Override
  public void updateLog(String standardPrefix, String inputPrefix) {
    Logger.processInputs(inputPrefix, inputs);
    io.updateLog(standardPrefix, inputPrefix);
    super.updateLog(standardPrefix, inputPrefix);
  }
}
