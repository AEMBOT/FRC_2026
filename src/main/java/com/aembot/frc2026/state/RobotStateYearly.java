package com.aembot.frc2026.state;

import com.aembot.lib.state.RobotState;
import com.aembot.lib.state.subsystems.intake.over_bumper.deploy.OverBumperIntakeDeployState;
import com.aembot.lib.state.subsystems.intake.over_bumper.run.OverBumperIntakeRollerState;

public class RobotStateYearly extends RobotState {
  // Ppl on the interwebs say this is good & thread safe
  private static final RobotStateYearly INSTANCE = new RobotStateYearly();

  protected OverBumperIntakeDeployState intakeDeployState = new OverBumperIntakeDeployState();
  protected OverBumperIntakeRollerState intakeRollerState = new OverBumperIntakeRollerState();

  public void updateIntakeDeployState(OverBumperIntakeDeployState state) {
    intakeDeployState = state;
  }

  public OverBumperIntakeDeployState getIntakeDeployState() {
    return intakeDeployState;
  }

  public void updateIntakeRollerState(OverBumperIntakeRollerState state) {
    intakeRollerState = state;
  }

  public OverBumperIntakeRollerState getIntakeRollerState() {
    return intakeRollerState;
  }

  public static RobotStateYearly get() {
    return INSTANCE;
  }
}
