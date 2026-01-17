package com.aembot.frc2026.state;

import com.aembot.lib.state.RobotState;

public class RobotStateYearly extends RobotState {
  private static RobotStateYearly instance;

  public static RobotStateYearly get() {
    if (instance == null) {
      instance = new RobotStateYearly();
    }
    return instance;
  }
}
