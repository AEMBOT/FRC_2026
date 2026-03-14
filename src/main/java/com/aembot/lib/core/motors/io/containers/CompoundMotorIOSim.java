package com.aembot.lib.core.motors.io.containers;

import com.aembot.lib.core.motors.interfaces.MotorIO;
import com.aembot.lib.core.motors.io.MotorIOTalonFXSim;
import edu.wpi.first.wpilibj.Notifier;
import java.util.ArrayList;
import java.util.List;

public class CompoundMotorIOSim<M extends MotorIO> extends CompoundMotorIO<M> {
  private Notifier kSimNotifier;
  private double kPeriodicSeconds = 0.005;

  public CompoundMotorIOSim() {
    super();
  }

  public CompoundMotorIOSim(List<M> motors) {
    super(motors);
  }

  @SafeVarargs
  public CompoundMotorIOSim(M... motors) {
    super(motors);
  }

  public CompoundMotorIOSim(double periodicSeconds) {
    this();
    this.kPeriodicSeconds = periodicSeconds;
  }

  public CompoundMotorIOSim(double periodicSeconds, List<M> motors) {
    this(motors);
    this.kPeriodicSeconds = periodicSeconds;
  }

  @SafeVarargs
  public CompoundMotorIOSim(double periodicSeconds, M... motors) {
    this(motors);
    this.kPeriodicSeconds = periodicSeconds;
  }

  @Override
  public void implementationSetup() {
    if (kMotors.size() > 0) {
      List<Runnable> updaters = new ArrayList<>();
      for (MotorIO motor : kMotors) {
        if (motor instanceof MotorIOTalonFXSim) {
          updaters.add(((MotorIOTalonFXSim) motor)::updateSimState);
        }
      }

      this.kSimNotifier =
          new Notifier(
              () -> {
                for (Runnable updater : updaters) {
                  updater.run();
                }
              });

      kSimNotifier.setName(kMotors.get(0).getName() + "Notifier");
      kSimNotifier.startPeriodic(kPeriodicSeconds);
    }
  }
}
