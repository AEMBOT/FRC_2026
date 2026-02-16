package com.aembot.frc2026.commands;

import com.aembot.frc2026.state.RobotStateYearly;
import com.aembot.frc2026.state.subsystems.indexer.IndexerCompoundState;
import com.aembot.frc2026.state.subsystems.indexer.IndexerCompoundState.IndexerRunState;
import com.aembot.frc2026.subsystems.indexerKicker.IndexerKickerSubsystem;
import com.aembot.frc2026.subsystems.indexerSelector.IndexerSelectorSubsystem;
import com.aembot.frc2026.subsystems.spindexer.SpindexerSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public final class IndexerCommands {
  /** Buffer in seconds added to the expected load time of game pieces for load timeout */
  private static final double LOAD_TIMEOUT_BUFFER = 2.0;

  private final SpindexerSubsystem spindexer;
  private final IndexerSelectorSubsystem selector;
  private final IndexerKickerSubsystem kicker;

  /**
   * A dummy subsystem for command requirements, ensuring we don't unintentionally run multiple
   * commands at the same time
   */
  private final SubsystemBase dummySubsystem =
      new SubsystemBase("Indexer Compound Dummy Subsystem") {};

  private static final IndexerCompoundState indexerCompoundState =
      RobotStateYearly.get().indexerCompoundState;

  public IndexerCommands(
      SpindexerSubsystem spindexerSubsystem,
      IndexerSelectorSubsystem indexerSelectorSubsystem,
      IndexerKickerSubsystem indexerKickerSubsystem) {
    this.spindexer = spindexerSubsystem;
    this.selector = indexerSelectorSubsystem;
    this.kicker = indexerKickerSubsystem;
  }

  /** InstantCommand to command the indexer to {@link IndexerRunState#OFF}. */
  public InstantCommand createDisableIndexerCommand() {
    return new InstantCommand(
        () -> indexerCompoundState.commandState(IndexerRunState.OFF), dummySubsystem);
  }

  /**
   * Command to command the indexer to {@link IndexerRunState#LOAD} until a game piece is detected
   * at the end of the selector. Nothing will happen upon termination. This is only intended for use
   * in command composition, and won't be very useful on its own.
   */
  public Command createSimpleLoadIndexerCommand() {
    return new RunCommand(
            () -> indexerCompoundState.commandState(IndexerRunState.LOAD), dummySubsystem)
        .until(indexerCompoundState::getGamePieceAtSelector);
  }

  /**
   * Command to run the indexer at {@link IndexerRunState#LOAD} until a game piece is detected, a
   * timeout is exceeded (Time it takes for a game piece to travel from intake to kicker base +
   * {@link #LOAD_TIMEOUT_BUFFER}), or the command is interrupted by another command requiring the
   * indexer compound. Commands the indexer to {@link IndexerRunState#OFF} afterwards.
   */
  public Command createLoadIndexerUntilTimeoutCommand() {
    return createSimpleLoadIndexerCommand()
        .withTimeout(
            LOAD_TIMEOUT_BUFFER
                + spindexer.kConfig.kGamePieceMoveTime
                + selector.kConfig.kGamePieceMoveTime)
        .andThen(createDisableIndexerCommand());
  }

  /**
   * Command to load game pieces from the spindexer to the selector, stopping short of the kicker.
   * Commands the indexer to {@link IndexerRunState#LOAD}. After termination, will schedule a
   * command to continue running the indexer at {@link IndexerRunState#LOAD} until a game piece is
   * detected, a timeout is exceeded (Time it takes for a game piece to travel from intake to kicker
   * base + {@link #LOAD_TIMEOUT_BUFFER}), or the command is interrupted by another command
   * requiring the indexer compound. Commands the indexer to {@link IndexerRunState#OFF} afterwards.
   */
  public Command createLoadIndexerCommand() {
    return createSimpleLoadIndexerCommand()
        .finallyDo(
            () -> CommandScheduler.getInstance().schedule(createLoadIndexerUntilTimeoutCommand()));
  }

  public Command createFeedIndexerCommand() {
    return new RunCommand(
            () -> indexerCompoundState.commandState(IndexerRunState.FIRE), dummySubsystem)
        .finallyDo(
            () -> CommandScheduler.getInstance().schedule(createLoadIndexerUntilTimeoutCommand()));
  }
}
