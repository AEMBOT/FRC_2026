package com.aembot.lib.core.motors.io.containers;

import com.aembot.lib.core.motors.interfaces.MotorIO;
import java.util.List;

public class CompoundMotorIOReplay<M extends MotorIO> extends CompoundMotorIO<M> {
  public CompoundMotorIOReplay() {
    super();
  }

  public CompoundMotorIOReplay(List<M> motors) {
    super(motors);
  }

  @SafeVarargs
  public CompoundMotorIOReplay(M... motors) {
    super(motors);
  }
}
