package com.aembot.lib.core.motors.io;

import com.aembot.lib.config.motors.SimulatedMotorConfiguration;
import com.aembot.lib.core.motors.visualization.SimulatedTalonFXVisualization;
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
  }

  @Override
  protected void setupMotorSim() {
    double gearRatio = config.kRealConfiguration.getGearRatio();
    var plant =
        LinearSystemId.createFlywheelSystem(
            config.kSimMotorConstants, config.kRealConfiguration.kMomentOfInertia, gearRatio);
    this.motorSim = new FlywheelSim(plant, config.kSimMotorConstants);

    visualization =
        new SimulatedTalonFXVisualization(
            Units.rotationsToDegrees(Double.MAX_VALUE),
            Units.rotationsToDegrees(Double.MAX_VALUE),
            Units.rotationsToDegrees(0.0));
  }

  @Override
  public void updateSimState() {

    inputs.SupplyVoltage = RobotController.getBatteryVoltage();

    simState.setSupplyVoltage(inputs.SupplyVoltage);

    inputs.SimVoltage = simState.getMotorVoltage();

    double timestamp = Timer.getFPGATimestamp();
    double dt = timestamp - lastUpdateTimestamp;
    lastUpdateTimestamp = timestamp;

    // TODO find place to store these values
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
        
    inputs.SimAccelerationUnits =
      config.kRealConfiguration.getMechanismRotationsToUnits(
        Units.radiansToRotations(motorSim.getAngularAccelerationRadPerSecSq())
      );
    inputs.RotorAcceleration = config.kRealConfiguration.getUnitsToRotorRotations(inputs.SimAccelerationUnits);
    

    inputs.SimPosUnits += (inputs.SimVelocityUnits / 60) * dt;
    inputs.RotorPosition += (inputs.RotorVelocity / 60) * dt;

    simState.setRotorAcceleration(inputs.RotorAcceleration);
    simState.setRawRotorPosition(inputs.RotorPosition);
    simState.setRotorVelocity(inputs.RotorVelocity);

    visualization.updateAngle(inputs.SimPosUnits);
  }
}
