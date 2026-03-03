package com.aembot.frc2026.state.subsystems.indexer;

import com.aembot.lib.core.logging.Loggable;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;
import org.littletonrobotics.junction.Logger;

/**
 * Represents the high-level state of the indexer compound subsystem. All fields and methods should
 * be thread-safe
 */
public class IndexerCompoundState implements Loggable {
  public enum IndexerStageRunState {
    /** The indexer stage is trying to push a game piece backwards */
    REVERSE,
    /** The indexer stage is trying to resist movement from one indexer stage into itself */
    RESIST,
    /** The indexer stage is not moving */
    OFF,
    /** The indexer stage is running forward, indexing a game piece */
    FORWARD
  }

  public enum IndexerRunState {
    /** All of the indexer stages are unmoving. */
    OFF(IndexerStageRunState.OFF, IndexerStageRunState.OFF, IndexerStageRunState.OFF),
    /** The indexer is loading game pieces, but not shooting yet */
    LOAD(IndexerStageRunState.FORWARD, IndexerStageRunState.FORWARD, IndexerStageRunState.RESIST),
    /** The indexer is loading game pieces and shooting them immediately */
    FIRE(IndexerStageRunState.FORWARD, IndexerStageRunState.FORWARD, IndexerStageRunState.FORWARD),
    /** Run all indexer stages backward */
    REVERSE(IndexerStageRunState.REVERSE, IndexerStageRunState.REVERSE, IndexerStageRunState.REVERSE),
    ;

    public final IndexerStageRunState SPINDEXER_DIRECTION;
    public final IndexerStageRunState SELECTOR_DIRECTION;
    public final IndexerStageRunState KICKER_DIRECTION;

    private IndexerRunState(
        IndexerStageRunState spindexerDirection,
        IndexerStageRunState selectorDirection,
        IndexerStageRunState kickerDirection) {
      this.SPINDEXER_DIRECTION = spindexerDirection;
      this.SELECTOR_DIRECTION = selectorDirection;
      this.KICKER_DIRECTION = kickerDirection;
    }
  }

  private final String kSpindexerName;
  private final String kSelectorName;
  private final String kKickerName;

  /**
   * @param spindexerName The name of the spindexer subsystem. Used for logging.
   * @param selectorName The name of the selector subsystem. Used for logging.
   * @param kickerName The name of the kicker subsystem. Used for logging.
   */
  public IndexerCompoundState(String spindexerName, String selectorName, String kickerName) {
    this.kSpindexerName = spindexerName;
    this.kSelectorName = selectorName;
    this.kKickerName = kickerName;
  }

  /** The commanded state of the indexer compound subsystem */
  public AtomicReference<IndexerRunState> commandedState =
      new AtomicReference<>(IndexerRunState.OFF);

  /** Whether or not a game piece has been detected at the end of the selector */
  public AtomicBoolean gamePieceInSelector = new AtomicBoolean(false);

  /** Command the indexer to run a given {@link IndexerRunState} */
  public void commandState(IndexerRunState state) {
    this.commandedState.set(state);
  }

  public void updateGamePieceInSelector(boolean value) {
    gamePieceInSelector.set(value);
  }

  /** Get whether or not a game piece has been detected at the end of the selector */
  public boolean getGamePieceInSelector() {
    return gamePieceInSelector.get();
  }

  /** Get the commanded state of the indexer compound subsystem */
  public IndexerRunState getCommandedState() {
    return commandedState.get();
  }

  /** Get the commanded state of the spindexer subsystem */
  public IndexerStageRunState getSpindexerCommandedState() {
    return getCommandedState().SPINDEXER_DIRECTION;
  }

  /** Get the commanded state of the selector subsystem */
  public IndexerStageRunState getSelectorCommandedState() {
    return getCommandedState().SELECTOR_DIRECTION;
  }

  /** Get the commanded state of the kicker subsystem */
  public IndexerStageRunState getKickerCommandedState() {
    return getCommandedState().KICKER_DIRECTION;
  }

  @Override
  public void updateLog(String standardPrefix, String inputPrefix) {
    Logger.recordOutput(standardPrefix + "/State", getCommandedState());
    Logger.recordOutput(
        standardPrefix + "/" + kSpindexerName + "/CommandedState", getSpindexerCommandedState());
    Logger.recordOutput(
        standardPrefix + "/" + kSelectorName + "/CommandedState", getSelectorCommandedState());
    Logger.recordOutput(
        standardPrefix + "/" + kKickerName + "/CommandedState", getKickerCommandedState());

    Logger.recordOutput(
        standardPrefix + "/" + kSelectorName + "/GamePieceDetected", getGamePieceInSelector());
  }
}
