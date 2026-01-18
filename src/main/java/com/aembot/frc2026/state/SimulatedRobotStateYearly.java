package com.aembot.frc2026.state;

import com.aembot.lib.state.SimulatedRobotState;

public class SimulatedRobotStateYearly extends SimulatedRobotState {
  private SimulatedRobotStateYearly() {
    // TODO Add apriltags to vision sim
  }

  private static SimulatedRobotStateYearly instance = null;

  public static SimulatedRobotStateYearly get() {
    if (instance == null) {
      instance = new SimulatedRobotStateYearly();
    }
    return instance;
  }
}
