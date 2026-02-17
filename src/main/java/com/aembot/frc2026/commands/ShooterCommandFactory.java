package com.aembot.frc2026.commands;

import com.aembot.lib.subsystems.hood.HoodSubsystem;

public final class ShooterCommandFactory {

  private final HoodSubsystem hood;

  public ShooterCommandFactory(HoodSubsystem hood) {
    this.hood = hood;
  }
}
