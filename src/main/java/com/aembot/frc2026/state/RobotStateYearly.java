package com.aembot.frc2026.state;

import com.aembot.lib.state.RobotState;

public class RobotStateYearly extends RobotState {
  // Ppl on the interwebs say this is good & thread safe
  private static final RobotStateYearly INSTANCE = new RobotStateYearly();

  public static RobotStateYearly get() {
    return INSTANCE;
  }
}
