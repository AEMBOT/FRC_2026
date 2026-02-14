package com.aembot.frc2026.state;

import com.aembot.lib.state.SimulatedRobotState;

public class SimulatedRobotStateYearly extends SimulatedRobotState {
  private SimulatedRobotStateYearly() {
    // TODO Add apriltags to vision sim
  }

  // Ppl on the interwebs say this is good & thread safe
  private static final SimulatedRobotStateYearly INSTANCE = new SimulatedRobotStateYearly();

  public static SimulatedRobotStateYearly get() {
    return INSTANCE;
  }
}
