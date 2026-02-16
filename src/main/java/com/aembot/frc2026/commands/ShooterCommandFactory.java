package com.aembot.frc2026.commands;

import com.aembot.frc2026.subsystems.turret.TurretSubsystem;
import com.aembot.lib.subsystems.hood.HoodSubsystem;

public final class ShooterCommandFactory {

  private final HoodSubsystem hood;
  private final TurretSubsystem turret;

  public ShooterCommandFactory(HoodSubsystem hood, TurretSubsystem turret) {
    this.hood = hood;
    this.turret = turret;
  }
}
