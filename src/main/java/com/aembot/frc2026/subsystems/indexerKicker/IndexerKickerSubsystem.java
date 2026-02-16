package com.aembot.frc2026.subsystems.indexerKicker;

import com.aembot.frc2026.config.subsystems.indexerKicker.IndexerKickerConfiguration;
import com.aembot.frc2026.subsystems.indexerKicker.io.IndexerKickerMechanismIO;
import com.aembot.lib.config.motors.MotorConfiguration;
import com.aembot.lib.core.motors.MotorInputs;
import com.aembot.lib.core.motors.interfaces.MotorIO;
import com.aembot.lib.subsystems.base.MotorSubsystem;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

/**
 * Indexer kicker subsystem that transports balls from the {@link
 * com.aembot.frc2026.subsystems.indexerSelector.IndexerSelectorSubsystem selector} to the shooter.
 * Stage 3/3 of the whole indexing system.
 */
public class IndexerKickerSubsystem
    extends MotorSubsystem<MotorInputs, MotorIO, MotorConfiguration<TalonFXConfiguration>> {
  private final IndexerKickerMechanismIO kMechanismIO;
  private final IndexerKickerMechanismInputs kMechanismInputs;

  private final IndexerKickerConfiguration kConfig;

  public IndexerKickerSubsystem(IndexerKickerConfiguration config, IndexerKickerMechanismIO io) {
    super(config.kName, new MotorInputs(), io.getMotor(), config.kMotorConfig);
    this.kMechanismIO = io;
    this.kConfig = config;
    this.kMechanismInputs = new IndexerKickerMechanismInputs();
  }

  @Override
  public void periodic() {
    super.periodic();

    kMechanismIO.updateInputs(kMechanismInputs);
  }

  /**
   * @return Command that will set the target velocity of the kicker to the velocity defined in
   *     {@link #kConfig}'s {@link IndexerKickerConfiguration#kTargetSpeedRPM}. Runs until
   *     termination. Note that this does not set target velocity to 0 upon termination.
   */
  public Command runKickerCommand() {
    return this.smartVelocitySetpointCommand(() -> kConfig.kTargetSpeedRPM / 60);
  }

  /**
   * Command to run the kicker in reverse, preventing balls from being pushed into the shooter
   *
   * @return Command that will set the target velocity of the kicker to the velocity defined in
   *     {@link #kConfig}'s {@link IndexerKickerConfiguration#kResistSpeedRPM}. Runs until
   *     termination. Note that this does not set target velocity to 0 upon termination.
   */
  public Command resistKickerCommand() {
    return this.smartVelocitySetpointCommand(() -> kConfig.kResistSpeedRPM / 60);
  }

  /**
   * @return Command that will set the target velocity of the kicker to 0. Runs until termination.
   */
  public Command stopKickerCommand() {
    return this.smartVelocitySetpointCommand(() -> 0);
  }

  @Override
  public void updateLog(String standardPrefix, String inputPrefix) {
    super.updateLog(standardPrefix, inputPrefix);

    Logger.processInputs(inputPrefix + "/AuxilaryInputs", kMechanismInputs);
  }
}
