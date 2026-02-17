package com.aembot.lib.subsystems.flywheel;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/** Inputs for the flywheel subsystem */
public class FlywheelInputs implements LoggableInputs {
  //   public double velocity = 0.0;

  @Override
  public void toLog(LogTable table) {
    // table.put("Velocity", velocity);
  }

  @Override
  public void fromLog(LogTable table) {
    // velocity = table.get("Velocity", velocity);
  }
}
