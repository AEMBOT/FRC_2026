package com.aembot.lib.subsystems.intake.generic.run.io;

import com.aembot.lib.config.subsystems.intake.generic.run.TalonFXIntakeRollerConfiguration;
import com.aembot.lib.core.motors.interfaces.MotorIO;
import com.aembot.lib.core.motors.io.MotorIOTalonFXSimFlywheel;
import com.aembot.lib.subsystems.intake.generic.run.IntakeRollerInputs;
import edu.wpi.first.wpilibj.Notifier;

/** simulated implementation for over the bumper intake roller io */
public class IntakeRollerSimIO implements IntakeRollerIO {

  /** Internal motor to use */
  private final MotorIOTalonFXSimFlywheel simMotor;

  /** Configuration to use */
  @SuppressWarnings("unused") // Currently unused but that may change in the future
  private final TalonFXIntakeRollerConfiguration config;

  /** Notifier to run the sim */
  private final Notifier simNotifier;

  /**
   * Create a new simulated IO layer for an over the bumper intake's gmae piece intaking system
   *
   * @param config The configuration to use for this io
   */
  public IntakeRollerSimIO(TalonFXIntakeRollerConfiguration config) {
    this.config = config;
    this.simMotor = new MotorIOTalonFXSimFlywheel(config.kSimMotorConfig);
    this.simNotifier = new Notifier(() -> simMotor.updateSimState());
    simNotifier.setName(config.kName + "Notifier");
    simNotifier.startPeriodic(0.005);
  }

  @Override
  public MotorIO getMotor() {
    return simMotor;
  }

  @Override
  public void updateInputs(IntakeRollerInputs inputs) {}

  @Override
  public void updateLog(String standardPrefix, String inputPrefix) {
    simMotor.logSim(standardPrefix, inputPrefix);
  }
}
