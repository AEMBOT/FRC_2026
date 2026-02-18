package com.aembot.lib.subsystems.flywheel.io;

import com.aembot.lib.config.subsystems.flywheel.TalonFXFlywheelConfiguration;
import com.aembot.lib.core.motors.interfaces.MotorIO;
import com.aembot.lib.core.motors.io.MotorIOTalonFXSim;
import com.aembot.lib.subsystems.flywheel.FlywheelInputs;
import edu.wpi.first.wpilibj.Notifier;

/** Flywheel IO implementation for simulation */
public class FlywheelSimIO extends FlywheelHardwareIO {
  private final MotorIOTalonFXSim leadMotorSim;
  private final Notifier simNotifier;

  public FlywheelSimIO(TalonFXFlywheelConfiguration flywheel) {
    super(flywheel);
    leadMotorSim = new MotorIOTalonFXSim(flywheel.kSimMotorConfig);
    this.simNotifier = new Notifier(() -> leadMotorSim.updateSimState());
    simNotifier.setName(flywheel.kName + "Notifier");
    simNotifier.startPeriodic(0.005);
  }
  ;

  @Override
  public MotorIO getMotor() {
    return leadMotorSim;
  }

  @Override
  public void updateInputs(FlywheelInputs inputs) {}

  @Override
  public void updateLog(String standardPrefix, String inputPrefix) {
    leadMotorSim.logSim(standardPrefix, inputPrefix);
  }
}
