package com.aembot.frc2026.subsystems.spindexer;

import com.aembot.frc2026.config.subsystems.spindexer.SpindexerConfiguration;
import com.aembot.lib.config.motors.MotorConfiguration;
import com.aembot.lib.core.motors.MotorInputs;
import com.aembot.lib.core.motors.interfaces.MotorIO;
import com.aembot.lib.subsystems.base.MotorSubsystem;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Spindexer subsystem that transports balls from the intake to the feeder. Stage 1 of the whole
 * indexing system.
 */
public class SpindexerSubsystem
    extends MotorSubsystem<MotorInputs, MotorIO, MotorConfiguration<TalonFXConfiguration>> {
  private final MotorIO kMotorIO;

  private final SpindexerConfiguration kConfig;

  public SpindexerSubsystem(SpindexerConfiguration config, MotorIO io) {
    super(new MotorInputs(), io, config.kMotorConfig);
    this.kMotorIO = io;
    this.kConfig = config;
  }

  /**
   * @return Command that will set the target velocity of the spindexer to the velocity defined in
   *     {@link #kConfig}'s {@link SpindexerConfiguration#kTargetSpeedRPM}. Runs until termination.
   *     Note that this does not set target velocity to 0 upon termination.
   */
  public Command runSpindexerCommand() {
    return this.smartVelocitySetpointCommand(() -> kConfig.kTargetSpeedRPM);
  }

  /**
   * @return Command that will set the target velocity of the spindexer to 0. Runs until
   *     termination.
   */
  public Command stopSpindexerCommand() {
    return this.smartVelocitySetpointCommand(() -> 0);
  }
}
