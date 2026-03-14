package com.aembot.lib.core.motors.io.containers;

import com.aembot.lib.core.motors.interfaces.MotorIO;
import java.util.List;

/**
 * Abstract class representing a container of multiple {@link MotorIO} instances. Sim
 * implementations should make sure motors are getting notifiers for updates.
 *
 * @see CompoundMotorIOReal
 * @see CompoundMotorIOReplay
 * @see CompoundMotorIOSim
 */
public abstract class CompoundMotorIO<M extends MotorIO> {
  public final List<M> kMotors;

  protected CompoundMotorIO() {
    this.kMotors = List.of();
    implementationSetup();
  }

  protected CompoundMotorIO(List<M> motors) {
    this.kMotors = List.copyOf(motors);
    implementationSetup();
  }

  @SafeVarargs
  protected CompoundMotorIO(M... motors) {
    this.kMotors = List.of(motors);
    implementationSetup();
  }

  protected void implementationSetup() {}

  /** Returns the motor at the specified index, or null if the index is out of bounds. */
  public M getMotor(int i) {
    try {
      return kMotors.get(i);
    } catch (IndexOutOfBoundsException e) {
      return null;
    }
  }
}
