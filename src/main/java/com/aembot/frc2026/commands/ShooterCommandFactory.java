package com.aembot.frc2026.commands;

import com.aembot.lib.subsystems.flywheel.FlywheelSubsystem;
import com.aembot.lib.subsystems.hood.HoodSubsystem;

public final class ShooterCommandFactory {

  private final HoodSubsystem hood;
  private final FlywheelSubsystem flywheel;

  public ShooterCommandFactory(HoodSubsystem hood, FlywheelSubsystem flywheelSubsystem) {
    this.hood = hood;
    this.flywheel = flywheelSubsystem;
  }
}
