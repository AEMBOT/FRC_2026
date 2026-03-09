package com.aembot.lib.subsystems.flywheel.io;

import com.aembot.lib.config.subsystems.flywheel.TalonFXFlywheelConfiguration;
import com.aembot.lib.core.motors.interfaces.MotorIO;
import com.aembot.lib.core.motors.io.MotorIOTalonFXSim;
import com.aembot.lib.subsystems.flywheel.FlywheelInputs;
import com.aembot.lib.tracing.Traced;
import edu.wpi.first.wpilibj.Notifier;

/** Flywheel IO implementation for simulation */
public class FlywheelSimIO extends FlywheelHardwareIO {
  private final MotorIOTalonFXSim simMotor;
  private final Notifier simNotifier;

  private final TalonFXFlywheelConfiguration config;

  public FlywheelSimIO(TalonFXFlywheelConfiguration flywheel) {
    super(flywheel);
    this.config = flywheel;
    simMotor = new MotorIOTalonFXSim(flywheel.kSimMotorConfig);
    this.simNotifier = new Notifier(() -> simMotor.updateSimState());
    simNotifier.setName(flywheel.kName + "Notifier");
    simNotifier.startPeriodic(0.005);
  }

  /**
   * Simulate an impulse load such as a game piece through a shooter as defined in {@link
   * TalonFXFlywheelConfiguration#kSimulateLoadImpulseFunction}
   */
  @Traced
  public void simulateImpulseLoad() {
    simMotor.forceSetMotorVelocity(
        config.kSimulateLoadImpulseFunction.apply(simMotor.getSimState().SimVelocityUnits));
  }

  @Override
  public MotorIO getMotor() {
    return simMotor;
  }

  @Override
  @Traced
  public void updateInputs(FlywheelInputs inputs) {}

  @Override
  @Traced
  public void updateLog(String standardPrefix, String inputPrefix) {
    simMotor.logSim(standardPrefix, inputPrefix);
  }
}
