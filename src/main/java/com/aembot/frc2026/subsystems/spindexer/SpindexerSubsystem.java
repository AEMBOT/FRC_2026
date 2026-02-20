package com.aembot.frc2026.subsystems.spindexer;

import com.aembot.frc2026.config.subsystems.indexerSelector.IndexerSelectorConfiguration;
import com.aembot.frc2026.config.subsystems.spindexer.SpindexerConfiguration;
import com.aembot.frc2026.state.subsystems.indexer.IndexerCompoundState.IndexerStageRunState;
import com.aembot.frc2026.subsystems.spindexer.io.SpindexerMechanismIO;
import com.aembot.lib.config.motors.MotorConfiguration;
import com.aembot.lib.core.motors.MotorInputs;
import com.aembot.lib.core.motors.interfaces.MotorIO;
import com.aembot.lib.subsystems.base.MotorSubsystem;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Spindexer subsystem that transports balls from the intake to the {@link
 * com.aembot.frc2026.subsystems.indexerSelector.IndexerSelectorSubsystem selector}. Stage 1/3 of
 * the whole indexing system.
 */
public class SpindexerSubsystem
    extends MotorSubsystem<MotorInputs, MotorIO, MotorConfiguration<TalonFXConfiguration>> {
  public final SpindexerConfiguration kConfig;

  private final SpindexerMechanismIO kMechanismIO;
  private final SpindexerMechanismInputs kMechanismInputs;

  private final Supplier<IndexerStageRunState> kCommandedRunStateSupplier;

  public SpindexerSubsystem(
      SpindexerConfiguration config,
      SpindexerMechanismIO io,
      Supplier<IndexerStageRunState> commandedRunStateSupplier) {
    super(config.kName, new MotorInputs(), io.getMotor(), config.kMotorConfig);
    this.kMechanismIO = io;
    this.kMechanismInputs = new SpindexerMechanismInputs();
    this.kConfig = config;
    this.kCommandedRunStateSupplier = commandedRunStateSupplier;

    setDefaultCommand(followCommandedState());
  }

  @Override
  public void periodic() {
    super.periodic();

    kMechanismIO.updateInputs(kMechanismInputs);
  }

  public Command followCommandedState() {
    return run(() -> {
          switch (kCommandedRunStateSupplier.get()) {
            case FORWARD:
              this.setSmartVelocitySetpointImpl(kConfig.kTargetSpeedRPM / 60);
              break;
            case REVERSE:
              this.setSmartVelocitySetpointImpl(-(kConfig.kTargetSpeedRPM / 60));
              break;
            default:
            case RESIST:
            case OFF:
              this.setSmartVelocitySetpointImpl(0);
              break;
          }
        })
        .withName("Following commanded state");
  }

  /**
   * @return Command that will set the target velocity of the spindexer to the velocity defined in
   *     {@link #kConfig}'s {@link IndexerSelectorConfiguration#kTargetSpeedRPM}. Runs until
   *     termination. Note that this does not set target velocity to 0 upon termination.
   */
  public Command runSpindexerCommand() {
    return this.smartVelocitySetpointCommand(() -> kConfig.kTargetSpeedRPM / 60)
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

    kMechanismIO.updateLog(standardPrefix, inputPrefix);
    Logger.processInputs(inputPrefix + "/AuxilaryInputs", kMechanismInputs);
  }
}
