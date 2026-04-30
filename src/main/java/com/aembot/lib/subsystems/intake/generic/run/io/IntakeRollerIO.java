package com.aembot.lib.subsystems.intake.generic.run.io;

import com.aembot.lib.core.logging.Loggable;
import com.aembot.lib.core.motors.interfaces.MotorIO;
import com.aembot.lib.subsystems.intake.generic.run.IntakeRollerInputs;

/** Base IO layer for an over the bumper intake roller subsystem */
public interface IntakeRollerIO extends Loggable {

  /**
   * @return The underlying motor that the io uses
   */
  public MotorIO getMotor();

  /**
   * Update the inputs of this io
   *
   * @param inputs New set of inputs
   */
  public void updateInputs(IntakeRollerInputs inputs);
}
