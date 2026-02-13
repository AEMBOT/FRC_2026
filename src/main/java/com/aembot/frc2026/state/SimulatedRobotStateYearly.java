package com.aembot.frc2026.state;

import com.aembot.lib.state.SimulatedRobotState;
import com.aembot.lib.state.subsystems.intake.over_bumper.deploy.OverBumperIntakeDeployState;
import com.aembot.lib.state.subsystems.intake.over_bumper.run.OverBumperIntakeRollerState;

public class SimulatedRobotStateYearly extends SimulatedRobotState {
  private SimulatedRobotStateYearly() {
    // TODO Add apriltags to vision sim
  }

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

  // Ppl on the interwebs say this is good & thread safe
  private static final SimulatedRobotStateYearly INSTANCE = new SimulatedRobotStateYearly();

  public static SimulatedRobotStateYearly get() {
    return INSTANCE;
  }
}
