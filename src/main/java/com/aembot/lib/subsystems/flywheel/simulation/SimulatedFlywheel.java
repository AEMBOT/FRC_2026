package com.aembot.lib.subsystems.flywheel.simulation;

import com.aembot.lib.config.motors.MotorFollowersConfiguration;
import com.aembot.lib.config.subsystems.flywheel.simulation.SimulatedFlywheelConfiguration;
import com.aembot.lib.core.logging.Loggable;
import com.aembot.lib.core.motors.interfaces.MotorIO.FollowDirection;
import com.aembot.lib.core.motors.io.MotorIOTalonFX;
import com.aembot.lib.core.motors.io.MotorIOTalonFXSim;
import com.aembot.lib.subsystems.flywheel.visualization.FlywheelVisualizer;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import org.littletonrobotics.junction.Logger;

/** Handles running a flywheel simulation */
public class SimulatedFlywheel implements Loggable {

  /** Tracks flywheel's current state */
  class SimulatedFlywheelInputs {
    double SupplyVoltage;
    double SimVoltage;
    double SimAngularVelocityRadPerSec;
    double SimAngularAccelerationRadPerSecSq;
    double RotorPositionRot;
    double RotorVelocityRotPerSec;
    double RotorAccelerationRotPerSecSq;
  }

  protected FlywheelVisualizer flywheelVis = new FlywheelVisualizer();
  protected SimulatedFlywheelInputs inputs = new SimulatedFlywheelInputs();

  // Set up motor instances and configurations
  protected MotorFollowersConfiguration<TalonFXConfiguration> config;
  protected SimulatedFlywheelConfiguration flywheelSimulationConfiguration;

  protected MotorIOTalonFXSim leadTalonSimulation;
  protected MotorIOTalonFXSim[] followerTalonSimulations;

  // The simulated flywheel
  protected FlywheelSim flywheelSimulation;

  // Update the simulation
  protected Notifier simNotifier = null;
  protected double lastUpdateTimestamp = 0.0;

  public SimulatedFlywheel(
      MotorFollowersConfiguration<TalonFXConfiguration> config,
      SimulatedFlywheelConfiguration simConfig) {
    this.config = config;
    this.flywheelSimulationConfiguration = simConfig;

    // Flywheel simulation builder with 0 simulated noise
    this.flywheelSimulation =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60Foc(config.followerConfigurations.size() + 1),
                simConfig.JKgMetersSquared,
                1 / simConfig.gearing),
            DCMotor.getKrakenX60Foc(config.followerConfigurations.size() + 1));

    // Leader talon setup
    leadTalonSimulation = new MotorIOTalonFXSim(config);

    // Create new simulated talons for each follower talon
    followerTalonSimulations = new MotorIOTalonFXSim[config.followerConfigurations.size()];
    for (int i = 0; i < config.followerConfigurations.size(); i++) {
      followerTalonSimulations[i] =
          new MotorIOTalonFXSim(config.followerConfigurations.get(i).config);
    }

    // Set up the way sim updates for proper behaviour
    simNotifier =
        new Notifier(
            () -> {
              updateSimState();
            });
    simNotifier.setName("FlywheelSimNotifier");
    simNotifier.startPeriodic(0.005);
  }

  @Override
  public void updateLog(String standardPrefix, String inputPrefix) {
    Logger.recordOutput(standardPrefix + "/Simulation/SupplyVoltage", inputs.SupplyVoltage);
    Logger.recordOutput(standardPrefix + "/Simulation/Voltage", inputs.SimVoltage);
    Logger.recordOutput(
        standardPrefix + "/Simulation/FlywheelVelocityRadPerSec",
        inputs.SimAngularVelocityRadPerSec);
    Logger.recordOutput(
        standardPrefix + "/Simulation/FlywheelAccelerationRadPerSecSq",
        inputs.SimAngularAccelerationRadPerSecSq);
    Logger.recordOutput(
        standardPrefix + "/Simulation/RotorVelocityRotPerSec", inputs.RotorVelocityRotPerSec);
    Logger.recordOutput(
        standardPrefix + "/Simulation/RotorAccelerationRotPerSecSq",
        inputs.RotorAccelerationRotPerSecSq);

    // Update the visualizer and the logs
    flywheelVis.updateVis(inputs.SimAngularVelocityRadPerSec);
    flywheelVis.updateLog(standardPrefix, inputPrefix);
  }

  protected void updateSimState() {
    TalonFXSimState simState = leadTalonSimulation.getSimState();

    inputs.SupplyVoltage = RobotController.getBatteryVoltage();
    simState.setSupplyVoltage(inputs.SupplyVoltage);

    // Input voltage into sim
    inputs.SimVoltage = simState.getMotorVoltage();
    // flywheelSimulation.setInput(inputs.SimVoltage);

    // Update flywheel by time
    double timestamp = Timer.getFPGATimestamp();
    double timeTraveled = timestamp - lastUpdateTimestamp;
    flywheelSimulation.update(timeTraveled);
    lastUpdateTimestamp = timestamp;

    // Collect sim data
    inputs.SimAngularVelocityRadPerSec = flywheelSimulation.getAngularVelocityRadPerSec();
    inputs.SimAngularAccelerationRadPerSecSq =
        flywheelSimulation.getAngularAccelerationRadPerSecSq();

    // Calculate rotor velocity, acceleration, and position
    inputs.RotorVelocityRotPerSec =
        inputs.SimAngularVelocityRadPerSec * flywheelSimulationConfiguration.radToRotorRatio;
    inputs.RotorAccelerationRotPerSecSq =
        inputs.SimAngularAccelerationRadPerSecSq * flywheelSimulationConfiguration.radToRotorRatio;
    inputs.RotorPositionRot =
        inputs.RotorPositionRot + (inputs.RotorVelocityRotPerSec * timeTraveled);

    // Update leader talon sim state rotor velocity, acceleration, and position
    simState.setRotorVelocity(inputs.RotorVelocityRotPerSec);
    simState.setRotorAcceleration(inputs.RotorAccelerationRotPerSecSq);
    simState.setRawRotorPosition(inputs.RotorPositionRot);

    // Update the follower talon sim states
    for (int i = 0; i < followerTalonSimulations.length; ++i) {
      if (this.config.followerConfigurations.get(i).followDirection == FollowDirection.SAME) {
        followerTalonSimulations[i]
            .getSimState()
            .setRawRotorPosition(inputs.RotorPositionRot * 1.0);
        followerTalonSimulations[i]
            .getSimState()
            .setRotorVelocity(inputs.RotorVelocityRotPerSec * 1.0);
        followerTalonSimulations[i]
            .getSimState()
            .setRotorAcceleration(inputs.RotorAccelerationRotPerSecSq * 1.0);
      } else {
        followerTalonSimulations[i]
            .getSimState()
            .setRawRotorPosition(inputs.RotorPositionRot * -1.0);
        followerTalonSimulations[i]
            .getSimState()
            .setRotorVelocity(inputs.RotorVelocityRotPerSec * -1.0);
        followerTalonSimulations[i]
            .getSimState()
            .setRotorAcceleration(inputs.RotorAccelerationRotPerSecSq * -1.0);
      }
    }
  }

  public MotorIOTalonFX getLeadTalon() {
    return leadTalonSimulation;
  }

  public MotorIOTalonFX[] getFollowerTalons() {
    return followerTalonSimulations;
  }

  public void setVoltage(double voltage) {
    flywheelSimulation.setInput(voltage);
  }
}
