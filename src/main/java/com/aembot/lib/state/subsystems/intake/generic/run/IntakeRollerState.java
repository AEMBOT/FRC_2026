package com.aembot.lib.state.subsystems.intake.generic.run;

import com.aembot.lib.core.logging.Loggable;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;
import org.littletonrobotics.junction.Logger;

public class IntakeRollerState implements Loggable {

  public AtomicReference<Double> angularVelocityUnitsPerMin;

  public AtomicBoolean isActive;

  @Override
  public void updateLog(String standardPrefix, String inputPrefix) {
    Logger.recordOutput(
        standardPrefix + "/angularVelocityUnitsPerMin", angularVelocityUnitsPerMin.get());
    Logger.recordOutput(standardPrefix + "/isActive", isActive.get());
  }
}
