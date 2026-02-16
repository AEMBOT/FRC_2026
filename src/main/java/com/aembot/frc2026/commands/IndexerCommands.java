package com.aembot.frc2026.commands;

import com.aembot.frc2026.subsystems.indexerKicker.IndexerKickerSubsystem;
import com.aembot.frc2026.subsystems.indexerSelector.IndexerSelectorSubsystem;
import com.aembot.frc2026.subsystems.spindexer.SpindexerSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public final class IndexerCommands {
  private final SpindexerSubsystem spindexer;
  private final IndexerSelectorSubsystem selector;
  private final IndexerKickerSubsystem kicker;

  public IndexerCommands(
      SpindexerSubsystem spindexerSubsystem,
      IndexerSelectorSubsystem indexerSelectorSubsystem,
      IndexerKickerSubsystem indexerKickerSubsystem) {
    this.spindexer = spindexerSubsystem;
    this.selector = indexerSelectorSubsystem;
    this.kicker = indexerKickerSubsystem;
  }

  /** Set the default commands of the spindexer, selector, and kicker to stop running. */
  public void defaultToStop() {
    spindexer.setDefaultCommand(spindexer.stopSpindexerCommand());
    selector.setDefaultCommand(selector.stopSelectorCommand());
    kicker.setDefaultCommand(kicker.stopKickerCommand());
  }

  /**
   * Run the spindexer and selector, with the kicker running in reverse to prevent balls from
   * entering the shooter. Runs until terminated. Does not set velocity to 0 upon termination
   * (unless that is the default command. See {@link #defaultToStop()})
   */
  public Command createRunIndexerCommand() {
    // Note: this assumes that the kicker is able to resist the selector, that it needs to resist
    // the selector, and that this is the desired behavior. Alternatively, we might stop the
    // selector after it detects a game piece with the TOF sensor.
    return new ParallelCommandGroup(
        spindexer.runSpindexerCommand(),
        selector.runSelectorCommand(),
        kicker.resistKickerCommand());
  }

  /**
   * Run all indexer subsystems forwards, feeding fuel from the intake to the shooter. Runs until
   * terminated. Does not set velocity to 0 upon termination (unless that is the default command.
   * See {@link #defaultToStop()})
   */
  public Command createFireCommand() {
    return new ParallelCommandGroup(
        spindexer.runSpindexerCommand(), selector.runSelectorCommand(), kicker.runKickerCommand());
  }
}
