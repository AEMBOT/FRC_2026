package com.aembot.lib.core.motors.io.containers;

import com.aembot.lib.core.motors.interfaces.MotorIO;
import java.util.List;

public class CompoundMotorIOReal<M extends MotorIO> extends CompoundMotorIO<M> {
  public CompoundMotorIOReal() {
    super();
  }

  public CompoundMotorIOReal(List<M> motors) {
    super(motors);
  }

  @SafeVarargs
  public CompoundMotorIOReal(M... motors) {
    super(motors);
  }
}
