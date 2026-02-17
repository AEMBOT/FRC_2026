package com.aembot.lib.config.subsystems.flywheel.simulation;

public class SimulatedFlywheelConfiguration {
  public double JKgMetersSquared;
  public double gearing;
  public double radToRotorRatio;

  public SimulatedFlywheelConfiguration withJKgMetersSq(double JKgMetersSquared) {
    this.JKgMetersSquared = JKgMetersSquared;
    return this;
  }

  public SimulatedFlywheelConfiguration withGearing(double gearing) {
    this.gearing = gearing;
    return this;
  }

  public SimulatedFlywheelConfiguration withRadToRotorRatio(double ratio) {
    this.radToRotorRatio = ratio;
    return this;
  }
}
