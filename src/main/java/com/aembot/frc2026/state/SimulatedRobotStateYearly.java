package com.aembot.frc2026.state;

import com.aembot.frc2026.constants.RobotRuntimeConstants;
import com.aembot.frc2026.constants.field.Field2026;
import com.aembot.frc2026.state.subsystems.indexer.SimulatedIndexerCompoundState;
import com.aembot.lib.state.SimulatedRobotState;
import com.aembot.lib.state.subsystems.intake.over_bumper.SimulatedOverBumperIntakeState;
import edu.wpi.first.math.geometry.Pose3d;
import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.Logger;

public class SimulatedRobotStateYearly extends SimulatedRobotState {
  private SimulatedRobotStateYearly() {
    visionSimulation.addAprilTags(Field2026.get().getFieldLayout());

    SimulatedArena.getInstance().placeGamePiecesOnField();
  }

  // Ppl on the interwebs say this is good & thread safe
  private static final SimulatedRobotStateYearly INSTANCE = new SimulatedRobotStateYearly();

  public static SimulatedRobotStateYearly get() {
    return INSTANCE;
  }

  public final SimulatedOverBumperIntakeState simulatedIntakeState =
      new SimulatedOverBumperIntakeState(
          () -> RobotStateYearly.get().intakeDeployState.get(),
          () -> RobotStateYearly.get().intakeRollerState.get(),
          RobotRuntimeConstants.ROBOT_CONFIG.getIntakeDeployConfig());

  public final SimulatedIndexerCompoundState simulatedIndexerCompoundState =
      new SimulatedIndexerCompoundState(RobotStateYearly.get().indexerCompoundState);

  @Override
  public void updateState() {
    super.updateState();

    simulatedIntakeState.update();
    if (simulatedIntakeState.pullGamePiece()) simulatedIndexerCompoundState.addSimulatedGamePiece();
    simulatedIndexerCompoundState.update();
  }

  @Override
  public void updateLog(String standardPrefix, String inputPrefix) {
    super.updateLog(standardPrefix, inputPrefix);

    // Get the positions of the fuel (both on the field and in the air)
    Pose3d[] fuelPoses = SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel");
    // Publish to telemetry using AdvantageKit
    Logger.recordOutput("SimulatedRobotState/FuelPositions", fuelPoses);

    simulatedIndexerCompoundState.updateLog("SimulatedRobotState/IndexerCompound", "");
    simulatedIntakeState.updateLog("SimulatedRobotState/Intake", "");
  }
}
