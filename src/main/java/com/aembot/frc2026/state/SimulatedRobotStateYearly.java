package com.aembot.frc2026.state;

import com.aembot.frc2026.constants.field.Field2026;
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
}
