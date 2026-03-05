package com.aembot.lib.subsystems.hood.io;

import com.aembot.lib.core.logging.Loggable;
import com.aembot.lib.core.motors.interfaces.MotorIO;
import com.aembot.lib.subsystems.hood.HoodInputs;

/** Base IO layer for the hood subsystem */
public interface HoodIO extends Loggable {

  /**
   * @return The underlying motor that this io uses
   */
  public MotorIO getMotor();

  /**
   * Update the inputs of this io
   *
   * @param inputs New set of inputs
   */
  public void updateInputs(HoodInputs inputs);
}
