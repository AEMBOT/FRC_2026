package com.aembot.frc2026.commands;

import com.aembot.lib.subsystems.intake.over_bumper.deploy.OverBumperIntakeDeploySubsystem;
import com.aembot.lib.subsystems.intake.over_bumper.run.OverBumperIntakeRollerSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public final class IntakeCommands {

  private final OverBumperIntakeDeploySubsystem deploy;
  private final OverBumperIntakeRollerSubsystem roller;

  public IntakeCommands(
      OverBumperIntakeDeploySubsystem deploy, OverBumperIntakeRollerSubsystem roller) {
    this.deploy = deploy;
    this.roller = roller;
  }

  public Command createUpCommand() {
    return deploy.putIntakeUpCommand();
  }

  public Command createDownCommand() {
    return deploy.putIntakeDownCommand();
  }

  public Command createZeroUpCommand() {
    return deploy.getZeroUpwardCommand();
  }

  public Command createZeroDownCommand() {
    return deploy.getZeroDownwardCommand();
  }

  public Command runIntakeCommand() {
    return roller.runRollerCommand();
  }

  public Command stopIntakeCommand() {
    return roller.smartVelocitySetpointCommand(() -> 0);
  }
}
