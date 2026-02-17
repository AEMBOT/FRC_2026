package com.aembot.frc2026.state;

import com.aembot.frc2026.constants.field.Field2026;
import com.aembot.frc2026.state.subsystems.indexer.SimulatedIndexerCompoundState;
import com.aembot.lib.state.SimulatedRobotState;

public class SimulatedRobotStateYearly extends SimulatedRobotState {
  private SimulatedRobotStateYearly() {
    visionSimulation.addAprilTags(Field2026.get().getFieldLayout());
  }

  // Ppl on the interwebs say this is good & thread safe
  private static final SimulatedRobotStateYearly INSTANCE = new SimulatedRobotStateYearly();

  public static SimulatedRobotStateYearly get() {
    return INSTANCE;
  }

  public final SimulatedIndexerCompoundState simulatedIndexerCompoundState =
      new SimulatedIndexerCompoundState(RobotStateYearly.get().indexerCompoundState);

  @Override
  public void updateState() {
    super.updateState();

    simulatedIndexerCompoundState.update();
  }

  @Override
  public void updateLog(String standardPrefix, String inputPrefix) {
    super.updateLog(standardPrefix, inputPrefix);

    simulatedIndexerCompoundState.updateLog("SimulatedRobotState/IndexerCompound", "");
  }
}
