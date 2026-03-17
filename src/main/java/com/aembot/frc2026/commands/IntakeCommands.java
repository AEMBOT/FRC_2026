package com.aembot.frc2026.commands;

import com.aembot.lib.subsystems.intake.generic.multimotor.IntakeRollerMultiMotorSubsystem;
import com.aembot.lib.subsystems.intake.over_bumper.deploy.OverBumperIntakeDeploySubsystem;
import com.aembot.lib.subsystems.premades.BinaryVoltageMotorFollowerSubsytem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RepeatCommand;

public final class IntakeCommands {

  private final OverBumperIntakeDeploySubsystem deploy;
  private final IntakeRollerMultiMotorSubsystem roller;
  private final BinaryVoltageMotorFollowerSubsytem wheels;

  public IntakeCommands(
      OverBumperIntakeDeploySubsystem deploy,
      IntakeRollerMultiMotorSubsystem roller,
      BinaryVoltageMotorFollowerSubsytem wheels) {
    this.deploy = deploy;
    this.roller = roller;
    this.wheels = wheels;
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

  public Command createRunIntakeCommand() {
    return roller.runRollerCommand().alongWith(wheels.runSystemCommand());
  }

  public Command createStopIntakeCommand() {
    return roller.stopRollerCommand().alongWith(wheels.stopSystemCommand());
  }

  public Command createFlickIntakeCommand() {
    return deploy.flickIntakeCommand();
  }

  public Command createContinuousFlickIntakeCommand() {
    return new RepeatCommand(createFlickIntakeCommand());
  }
}
