package com.aembot.lib.subsystems.intake.over_bumper.deploy.io;

import com.aembot.lib.config.subsystems.intake.overBumper.deploy.simulation.SimulatedOverBumperIntakeDeployConfiguration;
import com.aembot.lib.core.motors.interfaces.MotorIO;
import com.aembot.lib.core.motors.io.MotorIOTalonFXSim;
import com.aembot.lib.subsystems.intake.over_bumper.deploy.OverBumperIntakeDeployInputs;
import edu.wpi.first.wpilibj.Notifier;

/** Simulated over the bumper intake deployement IO */
public class OverBumperIntakeDeploySimIO implements OverBumperIntakeDeployIO {

  /** Simulated Motor IO */
  private final MotorIOTalonFXSim simMotor;

  /** Configuration to use for this io */
  @SuppressWarnings("unused") // Currently unused but that may change in the future
  private final SimulatedOverBumperIntakeDeployConfiguration config;

  /** Notifier for this simulation */
  private final Notifier simNotifier;

  /**
   * Constructs a new simulated over the bumper intake deployment IO
   *
   * @param config Configuration for this simulation
   */
  public OverBumperIntakeDeploySimIO(SimulatedOverBumperIntakeDeployConfiguration config) {
    this.config = config;
    this.simMotor = new MotorIOTalonFXSim(config.kSimMotorConfig);
    this.simNotifier = new Notifier(() -> simMotor.updateSimState());
    simNotifier.setName(config.kRealConfig.kName + "Notifier");
    simNotifier.startPeriodic(0.005);
  }

  @Override
  public MotorIO getMotor() {
    return simMotor;
  }

  @Override
  public void updateInputs(OverBumperIntakeDeployInputs inputs) {}

  @Override
  public void updateLog(String standardPrefix, String inputPrefix) {
    simMotor.logSim(standardPrefix, inputPrefix);
  }
}
