package com.aembot.lib.subsystems.intake.over_bumper.run.io;

import com.aembot.lib.core.logging.Loggable;
import com.aembot.lib.core.motors.interfaces.MotorIO;
import com.aembot.lib.subsystems.intake.over_bumper.run.OverBumperIntakeRunInputs;

/** Base IO layer for an over the bumper intake game piece intaking subsystem */
public interface OverBumperIntakeRunIO extends Loggable {

  /**
   * @return The underlying motor that the io uses
   */
  public MotorIO getMotor();

  /**
   * Update the inputs of this io
   *
   * @param inputs New set of inputs
   */
  public void updateInputs(OverBumperIntakeRunInputs inputs);
}
