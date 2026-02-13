package com.aembot.lib.subsystems.intake.over_bumper.run.io;

import com.aembot.lib.config.subsystems.intake.overBumper.run.simulation.SimulatedOverBumperIntakeRollerConfiguration;
import com.aembot.lib.core.motors.interfaces.MotorIO;
import com.aembot.lib.core.motors.io.MotorIOTalonFXSimFlywheel;
import com.aembot.lib.subsystems.intake.over_bumper.run.OverBumperIntakeRollerInputs;
import edu.wpi.first.wpilibj.Notifier;

/** simulated implementation for over the bumper intake roller io */
public class OverBumperIntakeRollerSimIO implements OverBumperIntakeRollerIO {

  /** Internal motor to use */
  private final MotorIOTalonFXSimFlywheel simMotor;

  /** Configuration to use */
  @SuppressWarnings("unused") // Currently unused but that may change in the future
  private final SimulatedOverBumperIntakeRollerConfiguration config;

  /** Notifier to run the sim */
  private final Notifier simNotifier;

  /**
   * Create a new simulated IO layer for an over the bumper intake's gmae piece intaking system
   *
   * @param config The configuration to use for this io
   */
  public OverBumperIntakeRollerSimIO(SimulatedOverBumperIntakeRollerConfiguration config) {
    this.config = config;
    this.simMotor = new MotorIOTalonFXSimFlywheel(config.kSimMotorConfig);
    this.simNotifier = new Notifier(() -> simMotor.updateSimState());
    simNotifier.setName(config.kRealConfig.kName + "Notifier");
    simNotifier.startPeriodic(0.005);
  }

  @Override
  public MotorIO getMotor() {
    return simMotor;
  }

  @Override
  public void updateInputs(OverBumperIntakeRollerInputs inputs) {}

  @Override
  public void updateLog(String standardPrefix, String inputPrefix) {
    simMotor.logSim(standardPrefix, inputPrefix);
  }
}
