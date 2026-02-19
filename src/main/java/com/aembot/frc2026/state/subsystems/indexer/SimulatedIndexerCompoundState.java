package com.aembot.frc2026.state.subsystems.indexer;

import com.aembot.frc2026.constants.RobotRuntimeConstants;
import com.aembot.frc2026.state.subsystems.indexer.IndexerCompoundState.IndexerStageRunState;
import com.aembot.lib.config.sensors.timeOfFlight.TimeOfFlightConfiguration;
import com.aembot.lib.core.logging.Loggable;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.Consumer;
import org.littletonrobotics.junction.Logger;

/**
 * Class for simulating the indexer compound subsystem. The simulation is very simple and rather
 * unrealistic, but should be good enough for testing simple logic
 */
public class SimulatedIndexerCompoundState implements Loggable {
  enum IndexerStage {
    /**
     * The first stage of the indexer. Has the largest capacity, as this also includes the hopper.
     */
    SPINDEXER(
        RobotRuntimeConstants.ROBOT_CONFIG.getSpindexerConfiguration().kGamePieceMoveTime,
        RobotRuntimeConstants.ROBOT_CONFIG.getSpindexerConfiguration().kGamePieceCapacity),
    /** Second stage of the indexer. Pulls game pieces from the spindexer up to the kicker */
    SELECTOR(
        RobotRuntimeConstants.ROBOT_CONFIG.getIndexerSelectorConfiguration().kGamePieceMoveTime,
        RobotRuntimeConstants.ROBOT_CONFIG.getIndexerSelectorConfiguration().kGamePieceCapacity),
    /** Third stage of the indexer. Feeds game pieces to the shooter. */
    KICKER(
        RobotRuntimeConstants.ROBOT_CONFIG.getIndexerKickerConfiguration().kGamePieceMoveTime,
        RobotRuntimeConstants.ROBOT_CONFIG.getIndexerKickerConfiguration().kGamePieceCapacity),
    ;

    public final double kTimeThroughStage;
    public final int kGamePieceCapacity;

    private IndexerStage(double timeThroughStage, int capacity) {
      this.kTimeThroughStage = timeThroughStage;
      this.kGamePieceCapacity = capacity;
    }
  }

  static class IndexerSimulatedGamePiece {
    public double timerStartTimeSeconds;

    public IndexerSimulatedGamePiece() {
      this.timerStartTimeSeconds = Timer.getFPGATimestamp();
    }
  }

  private final IndexerCompoundState kRealIndexerState;

  /**
   * Consumer that consumes the simulated distance detected by the selector's time of flight sensor.
   */
  private Consumer<Double> kSelectorTOFDistanceConsumer;

  private final Map<IndexerStage, List<IndexerSimulatedGamePiece>> gamePiecesByStage;

  public SimulatedIndexerCompoundState(IndexerCompoundState realIndexerState) {
    this.kRealIndexerState = realIndexerState;

    this.gamePiecesByStage = new EnumMap<>(IndexerStage.class);
    for (IndexerStage stage : IndexerStage.values()) {
      gamePiecesByStage.put(stage, new ArrayList<>());
    }
  }

  public void setSelectorTimeOfFlightDistanceConsumer(Consumer<Double> consumer) {
    this.kSelectorTOFDistanceConsumer = consumer;
  }

  public void update() {
    moveGamePieces();

    TimeOfFlightConfiguration selectorTOFConfig =
        RobotRuntimeConstants.ROBOT_CONFIG.getIndexerSelectorConfiguration().kTimeOfFlightConfig;

    if (kSelectorTOFDistanceConsumer != null) {
      // Set the tof sensor to detect a game piece if there's one in the kicker. Technically the
      // sensor would be tripped at the end of the selector, but this is a lot easier with the way
      // this sim works, and shouldn't affect too much
      if (getGamePiecesFor(IndexerStage.KICKER).size() >= 1) {
        kSelectorTOFDistanceConsumer.accept(
            selectorTOFConfig.kDetectionThresholdMeters
                - selectorTOFConfig.kDetectionHysteresisMeters
                    * 1.01 // small nudge to get past hysteresis
            );
      } else {
        kSelectorTOFDistanceConsumer.accept(65.535); // Max value of a can range
      }
    }
  }

  /**
   * Add a game piece to the indexer simulation. This should be called when we intake a game piece
   * in sim.
   *
   * @return True if successful, false if failed because the hopper is at capacity.
   */
  public boolean addSimulatedGamePiece() {
    if (getGamePiecesFor(IndexerStage.SPINDEXER).size()
        < IndexerStage.SPINDEXER.kGamePieceCapacity) {
      getGamePiecesFor(IndexerStage.SPINDEXER).add(new IndexerSimulatedGamePiece());
      return true;
    } else {
      return false;
    }
  }

  /** Check that there's room in the indexer to intake a game piece */
  public boolean getRoomInIndexer() {
    return getGamePiecesFor(IndexerStage.SPINDEXER).size()
        < IndexerStage.SPINDEXER.kGamePieceCapacity;
  }

  /**
   * Pull a game piece from the kicker
   *
   * @return
   */
  public Optional<IndexerSimulatedGamePiece> pullFromKicker() {
    List<IndexerSimulatedGamePiece> gamePieces = getGamePiecesFor(IndexerStage.KICKER);
    if (!gamePieces.isEmpty()
        && getRunStateFor(IndexerStage.KICKER) == IndexerStageRunState.FORWARD) {
      // This doesn't really need to be FIFO since we don't rlly store meaningful data in these
      // objects, but we might later
      return Optional.of(gamePieces.remove(gamePieces.size() - 1));
    } else {
      return Optional.empty();
    }
  }

  /** Get the list of simulated game pieces for the given stage */
  private List<IndexerSimulatedGamePiece> getGamePiecesFor(IndexerStage stage) {
    return gamePiecesByStage.get(stage);
  }

  public List<Transform3d> getRenderedGamePiecePositions() {
    List<Transform3d> positions = new ArrayList<>();

    for (IndexerStage stage : IndexerStage.values()) {
      Transform3d[] stagePositions = getGamePiecePositionsFor(stage);
      if (stagePositions.length == 0) continue;
      for (int i = 0; i < getGamePiecesFor(stage).size(); i++) {
        positions.add(stagePositions[i % stagePositions.length]);
      }
    }

    return positions;
  }

  private void moveGamePieces() {
    IndexerStage[] indexerStages = IndexerStage.values();

    for (int i = 0; i < indexerStages.length; i++) {
      IndexerStage stage = indexerStages[i];

      IndexerStageRunState runState = getRunStateFor(stage);

      // The stage that this stage is feeding into. Null if it's feeding out of the indexer.
      IndexerStage nextStage;
      if (runState == null) continue; // Ie. shooter queue
      switch (runState) {
        case FORWARD:
          // if there's a stage after this, that one. Otherwise null
          nextStage = (indexerStages.length - 1 >= i + 1) ? indexerStages[i + 1] : null;
          break;
        case REVERSE:
          // if there's a stage before this, that one. Otherwise null
          nextStage = (i - 1 >= 0) ? indexerStages[i - 1] : null;
          break;
        default:
        case RESIST:
        case OFF:
          for (IndexerSimulatedGamePiece gamePiece : getGamePiecesFor(stage)) {
            // Reset the timer because the gamepiece isn't being moved
            gamePiece.timerStartTimeSeconds = Timer.getFPGATimestamp();
          }
          continue;
      }

      /**
       * List of game pieces to push to the next indexer stage. We do this in a seperate loop, bcuz
       * java (reasonably) doesn't take kindly to removing items from a list we're iterating over :p
       */
      List<IndexerSimulatedGamePiece> toPush = new ArrayList<>();

      for (IndexerSimulatedGamePiece gamePiece : getGamePiecesFor(stage)) {
        if (nextStage == null
            || getGamePiecesFor(nextStage).size() + toPush.size() >= nextStage.kGamePieceCapacity) {
          break;
        }

        if (Timer.getFPGATimestamp() - gamePiece.timerStartTimeSeconds >= stage.kTimeThroughStage) {
          toPush.add(gamePiece);
        }
      }

      for (IndexerSimulatedGamePiece gamePiece : toPush) {
        getGamePiecesFor(stage).remove(gamePiece);
        getGamePiecesFor(nextStage).add(gamePiece);
        gamePiece.timerStartTimeSeconds = Timer.getFPGATimestamp();
      }
    }
  }

  private IndexerStageRunState getRunStateFor(IndexerStage stage) {
    switch (stage) {
      case SPINDEXER:
        return kRealIndexerState.getSpindexerCommandedState();
      case SELECTOR:
        return kRealIndexerState.getSelectorCommandedState();
      case KICKER:
        return kRealIndexerState.getKickerCommandedState();
      default:
        return null;
    }
  }

  private Transform3d[] getGamePiecePositionsFor(IndexerStage stage) {
    switch (stage) {
      case SPINDEXER:
        return RobotRuntimeConstants.ROBOT_CONFIG.getSpindexerConfiguration().kGamePiecePositions;
      case SELECTOR:
        return RobotRuntimeConstants.ROBOT_CONFIG.getIndexerSelectorConfiguration()
            .kGamePiecePositions;
      case KICKER:
        return RobotRuntimeConstants.ROBOT_CONFIG.getIndexerKickerConfiguration()
            .kGamePiecePositions;
      default:
        return new Transform3d[0];
    }
  }

  @Override
  public void updateLog(String standardPrefix, String inputPrefix) {
    for (IndexerStage stage : IndexerStage.values()) {
      Logger.recordOutput(
          standardPrefix + "/" + stage.toString() + "/SimulatedGamePieces",
          getGamePiecesFor(stage).size());
    }
  }
}
