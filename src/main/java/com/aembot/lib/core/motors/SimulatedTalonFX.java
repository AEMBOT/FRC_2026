package com.aembot.lib.core.motors;

import com.aembot.lib.config.motors.factories.SimulatedMotorConfiguration;
import com.aembot.lib.core.logging.Loggable;
import com.aembot.lib.core.motors.io.MotorIOTalonFXSim;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.littletonrobotics.junction.Logger;

public class SimulatedTalonFX implements Loggable {

  class SimulatedTalonFXInputs {
    double SupplyVoltage;
    double SupplyCurrent;
    double SimVoltage;
    double SimPosDegrees;
    double SimVelocityDegrees;
    double RotorPosition;
    double RotorVelocity;
  }

  protected SimulatedTalonFXInputs inputs = new SimulatedTalonFXInputs();

  protected SimulatedMotorConfiguration<TalonFXConfiguration> config;

  protected MotorIOTalonFXSim motorIO;

  protected DCMotorSim motorSim;

  protected Notifier simNotifier = null;

  protected double lastUpdateTimestamp = 0.0;

  public SimulatedTalonFX(SimulatedMotorConfiguration<TalonFXConfiguration> config) {
    this.config = config;

    setupMotorSim();

    setupMotor();

    motorIO
        .getSimState()
        .setRawRotorPosition(
            config.kRealConfiguration.getUnitsToRotorRotations(config.kStartingRotationDegrees));

    lastUpdateTimestamp = Timer.getFPGATimestamp();

    simNotifier = new Notifier(() -> updateSimState());
    simNotifier.setName(config.toString() + "SimNotifier");
    simNotifier.startPeriodic(0.005);
  }

  private void setupMotorSim() {
    var plant =
        LinearSystemId.createDCMotorSystem(
            config.kSimMotorConstants,
            config.kRealConfiguration.kMomentOfInertia,
            1 / config.kRealConfiguration.kUnitToRotorRotationRatio);
    this.motorSim = new DCMotorSim(plant, config.kSimMotorConstants);

    double startAngle = Units.degreesToRotations(config.kStartingRotationDegrees);
    motorSim.setAngle(Units.rotationsToRadians(startAngle));
  }

  private void setupMotor() {
    motorIO = new MotorIOTalonFXSim(config.kRealConfiguration);
  }

  protected void updateSimState() {
    System.out.println("simstate updated");
    TalonFXSimState simState = motorIO.getSimState();

    inputs.SupplyVoltage = simState.getSupplyCurrent();

    inputs.SimVoltage = simState.getMotorVoltage();

    motorSim.setInputVoltage(inputs.SimVoltage);

    double timestamp = Timer.getFPGATimestamp();
    double dt = timestamp - lastUpdateTimestamp;
    lastUpdateTimestamp = timestamp;

    if (dt > 0.1) {
      dt = 0.005;
    }
    if (dt <= 0) {
      dt = 0.005;
    }

    motorSim.update(dt);

    double rawPosition = motorSim.getAngularPositionRad();

    double minAngleRad = Units.degreesToRadians(config.kRealConfiguration.kMinPositionUnits);
    double maxAngleRad = Units.degreesToRadians(config.kRealConfiguration.kMaxPositionUnits);

    double clampedPosition = MathUtil.clamp(rawPosition, minAngleRad, maxAngleRad);

    if (rawPosition != clampedPosition) {
      motorSim.setAngle(rawPosition);
      double currentVelocity = motorSim.getAngularVelocityRadPerSec();
      if ((rawPosition < minAngleRad && currentVelocity < 0)
          || (rawPosition > maxAngleRad && currentVelocity > 0)) {
        motorSim.setAngularVelocity(0);
      }
    }

    inputs.SimPosDegrees = Units.radiansToDegrees(rawPosition);
    inputs.SimVelocityDegrees = Units.radiansToDegrees(motorSim.getAngularVelocityRadPerSec());

    inputs.RotorPosition = config.kRealConfiguration.getUnitsToRotorRotations(inputs.SimPosDegrees);
    inputs.RotorVelocity =
        config.kRealConfiguration.getUnitsToRotorRotations(inputs.SimVelocityDegrees);

    simState.setRawRotorPosition(inputs.RotorPosition);
    simState.setRotorVelocity(inputs.RotorVelocity);
  }

  @Override
  public void updateLog(String standardPrefix, String inputPrefix) {
    Logger.recordOutput(standardPrefix + "/Simulation/SupplyVoltage", inputs.SupplyVoltage);
    Logger.recordOutput(standardPrefix + "/Simulation/SupplyCurrent", inputs.SupplyCurrent);
    Logger.recordOutput(standardPrefix + "/Simulation/Voltage", inputs.SimVoltage);
    Logger.recordOutput(standardPrefix + "/Simulation/PositionDegrees", inputs.SimPosDegrees);
    Logger.recordOutput(
        standardPrefix + "/Simulation/VelocityRadPerSec", inputs.SimVelocityDegrees);
    Logger.recordOutput(standardPrefix + "/Simulation/RotorPosition", inputs.RotorPosition);
    Logger.recordOutput(standardPrefix + "/Simulation/RotorVelocity", inputs.RotorVelocity);
  }
}
