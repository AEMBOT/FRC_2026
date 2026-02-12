package com.aembot.lib.core.motors.io;

import com.aembot.lib.config.motors.SimulatedMotorConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

/** Extends MotorIOTalonFXSim to use FlywheelSim instead of basic DCMotorSim */
public class MotorIOTalonFXSimFlywheel extends MotorIOTalonFXSim {

  protected FlywheelSim motorSim;

  public MotorIOTalonFXSimFlywheel(
      SimulatedMotorConfiguration<TalonFXConfiguration> simulatedMotorConfig) {
    super(simulatedMotorConfig);

    setupMotorSim();
  }

  private void setupMotorSim() {
    double gearRatio = config.kRealConfiguration.getGearRatio();
    var plant =
        LinearSystemId.createFlywheelSystem(
            config.kSimMotorConstants, config.kRealConfiguration.kMomentOfInertia, gearRatio);
    this.motorSim = new FlywheelSim(plant, config.kSimMotorConstants);
  }

  @Override
  public void updateSimState() {

    inputs.SupplyVoltage = RobotController.getBatteryVoltage();

    simState.setSupplyVoltage(inputs.SupplyVoltage);

    inputs.SimVoltage = simState.getMotorVoltage();

    double timestamp = Timer.getFPGATimestamp();
    double dt = timestamp - lastUpdateTimestamp;
    lastUpdateTimestamp = timestamp;

    if (dt > 0.1) {
      dt = 0.005;
    }
    if (dt <= 0) {
      dt = 0.005;
    }

    motorSim.setInputVoltage(inputs.SimVoltage);
    motorSim.update(dt);

    inputs.SimVelocityUnits =
        config.kRealConfiguration.getMechanismRotationsToUnits(
            Units.radiansPerSecondToRotationsPerMinute(motorSim.getAngularVelocityRadPerSec()));
    inputs.RotorVelocity =
        config.kRealConfiguration.getUnitsToMechanismRotations(inputs.SimVelocityUnits);

    inputs.SimPosUnits += inputs.SimVelocityUnits * dt / 60;
    inputs.RotorPosition += inputs.RotorVelocity * dt / 60;

    simState.setRawRotorPosition(inputs.RotorPosition);
    simState.setRotorVelocity(inputs.RotorVelocity);
  }
}
