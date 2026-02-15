package com.aembot.frc2026.subsystems.spindexer;

import com.aembot.frc2026.config.subsystems.indexerSelector.IndexerSelectorConfiguration;
import com.aembot.frc2026.config.subsystems.spindexer.SpindexerConfiguration;
import com.aembot.frc2026.subsystems.spindexer.io.SpindexerMechanismIO;
import com.aembot.lib.config.motors.MotorConfiguration;
import com.aembot.lib.core.motors.MotorInputs;
import com.aembot.lib.core.motors.interfaces.MotorIO;
import com.aembot.lib.subsystems.base.MotorSubsystem;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

/**
 * Spindexer subsystem that transports balls from the intake to the {@link
 * com.aembot.frc2026.subsystems.indexerSelector.IndexerSelectorSubsystem selector}. Stage 1/3 of
 * the whole indexing system.
 */
public class SpindexerSubsystem
    extends MotorSubsystem<MotorInputs, MotorIO, MotorConfiguration<TalonFXConfiguration>> {
  private final SpindexerMechanismIO kMechanismIO;
  private final SpindexerMechanismInputs kMechanismInputs;

  private final SpindexerConfiguration kConfig;

  public SpindexerSubsystem(SpindexerConfiguration config, SpindexerMechanismIO io) {
    super(config.kName, new MotorInputs(), io.getMotor(), config.kMotorConfig);
    this.kMechanismIO = io;
    this.kMechanismInputs = new SpindexerMechanismInputs();
    this.kConfig = config;
  }

  @Override
  public void periodic() {
    super.periodic();

    kMechanismIO.updateInputs(kMechanismInputs);
  }

  /**
   * @return Command that will set the target velocity of the spindexer to the velocity defined in
   *     {@link #kConfig}'s {@link IndexerSelectorConfiguration#kTargetSpeedRPM}. Runs until
   *     termination. Note that this does not set target velocity to 0 upon termination.
   */
  public Command runSpindexerCommand() {
    return this.smartVelocitySetpointCommand(() -> kConfig.kTargetSpeedRPM)
        .withName("SpindexerRunning");
  }

  /**
   * @return Command that will set the target velocity of the spindexer to 0. Runs until
   *     termination.
   */
  public Command stopSpindexerCommand() {
    return this.smartVelocitySetpointCommand(() -> 0).withName("SpindexerStopped");
  }

  @Override
  public void updateLog(String standardPrefix, String inputPrefix) {
    super.updateLog(standardPrefix, inputPrefix);

    Logger.processInputs(inputPrefix + "/AuxilaryInputs", kMechanismInputs);
  }
}
