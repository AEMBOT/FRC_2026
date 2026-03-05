package com.aembot.frc2026.subsystems.indexerSelector;

import com.aembot.frc2026.config.subsystems.indexerSelector.IndexerSelectorConfiguration;
import com.aembot.frc2026.state.subsystems.indexer.IndexerCompoundState.IndexerStageRunState;
import com.aembot.frc2026.subsystems.indexerSelector.io.IndexerSelectorMechanismIO;
import com.aembot.lib.config.motors.MotorConfiguration;
import com.aembot.lib.core.motors.MotorInputs;
import com.aembot.lib.core.motors.interfaces.MotorIO;
import com.aembot.lib.core.sensors.timeOfFlight.TimeOfFlightSensor;
import com.aembot.lib.core.sensors.timeOfFlight.interfaces.TimeOfFlightIO;
import com.aembot.lib.subsystems.base.MotorSubsystem;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Indexer selector subsystem that transports balls from the {@link
 * com.aembot.frc2026.subsystems.spindexer.SpindexerSubsystem spindexer} to the {@link
 * com.aembot.frc2026.subsystems.indexerKicker.IndexerKickerSubsystem kicker}. Stage 2/3 of the
 * whole indexing system.
 */
public class IndexerSelectorSubsystem
    extends MotorSubsystem<MotorInputs, MotorIO, MotorConfiguration<TalonFXConfiguration>> {
  public final IndexerSelectorConfiguration kConfig;

  private final IndexerSelectorMechanismIO kMechanismIO;
  private final IndexerSelectorMechanismInputs kMechanismInputs;

  private final TimeOfFlightSensor kTimeOfFlightSensor;

  private final Supplier<IndexerStageRunState> kCommandedRunStateSupplier;
  private final Consumer<Boolean> kGamePieceDetectedConsumer;

  public IndexerSelectorSubsystem(
      IndexerSelectorConfiguration config,
      IndexerSelectorMechanismIO mechanismIO,
      TimeOfFlightIO timeOfFlightIO,
      Supplier<IndexerStageRunState> commandedRunStateSupplier,
      Consumer<Boolean> gamePieceDetectedConsumer) {
    super(config.kName, new MotorInputs(), mechanismIO.getMotor(), config.kMotorConfig);
    this.kMechanismIO = mechanismIO;
    this.kMechanismInputs = new IndexerSelectorMechanismInputs();
    this.kTimeOfFlightSensor =
        new TimeOfFlightSensor(timeOfFlightIO, logPrefixStandard, logPrefixInput);
    this.kConfig = config;

    this.kCommandedRunStateSupplier = commandedRunStateSupplier;
    this.kGamePieceDetectedConsumer = gamePieceDetectedConsumer;

    setDefaultCommand(followCommandedState());
  }

  @Override
  public void periodic() {
    double timestamp = Timer.getFPGATimestamp();

    super.periodic();

    kTimeOfFlightSensor.update();
    kMechanismIO.updateInputs(kMechanismInputs);

    kGamePieceDetectedConsumer.accept(kTimeOfFlightSensor.getObjectDetected());

    // Log latency with time between periodic being called and finishing
    Logger.recordOutput(
        logPrefixStandard + "/LatencyPeriodicMS", (Timer.getFPGATimestamp() - timestamp) * 1000);
  }

  public Command followCommandedState() {
    return run(() -> {
          switch (kCommandedRunStateSupplier.get()) {
            case FORWARD:
              this.setVoltageImpl(kConfig.kRunVoltage);
              break;
            case REVERSE:
              this.setVoltageImpl(-kConfig.kRunVoltage);
              break;
            default:
            case RESIST:
            case OFF:
              this.setVoltageImpl(0);
              break;
          }
        })
        .withName("Following commanded state");
  }

  /**
   * @return Command that will set the target velocity of the spindexer to the velocity defined in
   *     {@link #kConfig}'s {@link IndexerSelectorConfiguration#kRunVoltage}. Runs until
   *     termination. Note that this does not set target velocity to 0 upon termination.
   */
  public Command runSelectorCommand() {
    return this.smartVelocitySetpointCommand(() -> kConfig.kRunVoltage / 60);
  }

  /**
   * @return Command that will set the target velocity of the spindexer to 0. Runs until
   *     termination.
   */
  public Command stopSelectorCommand() {
    return this.smartVelocitySetpointCommand(() -> 0);
  }

  public boolean getGamePieceDetected() {
    return kTimeOfFlightSensor.getObjectDetected();
  }

  @Override
  public void updateLog(String standardPrefix, String inputPrefix) {
    super.updateLog(standardPrefix, inputPrefix);

    kMechanismIO.updateLog(standardPrefix, inputPrefix);
    Logger.processInputs(inputPrefix + "/AuxilaryInputs", kMechanismInputs);
  }
}
