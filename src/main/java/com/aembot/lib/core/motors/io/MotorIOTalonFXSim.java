package com.aembot.lib.core.motors.io;

import com.aembot.lib.config.motors.SimulatedMotorConfiguration;
import com.aembot.lib.core.can.CANDeviceID;
import com.aembot.lib.core.motors.visualization.SimulatedTalonFXVisualization;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;
import org.littletonrobotics.junction.Logger;

/** IO implementation for a simulated TalonFX */
public class MotorIOTalonFXSim extends MotorIOTalonFX implements SimulatedMotorController {
  /** Helper class for simulated talon fx data */
  public class SimulatedTalonFXState {
    public double SupplyVoltage;
    public double SimVoltage;
    public double SimPosUnits;
    public double SimVelocityUnits;
    public double SimAccelerationUnits;
    public double RotorPosition;
    public double RotorVelocity;
    public double RotorAcceleration;
  }

  /** Sim state of the talonfx */
  protected final TalonFXSimState simState;

  public final Notifier notifer = new Notifier(this::updateSimState);

  /** Inputs to the simulated talonfx */
  protected SimulatedTalonFXState inputs = new SimulatedTalonFXState();

  /** Sim configuration */
  protected SimulatedMotorConfiguration<TalonFXConfiguration> config;

  /** Internal simulation of the motor */
  protected DCMotorSim motorSim;

  /** Time of the last sim update, used to calculate delta time */
  protected double lastUpdateTimestamp = 0.0;

  /** Visualization of the simulated motor */
  protected SimulatedTalonFXVisualization visualization;

  /**
   * Create a new TalonFX Sim IO using a ServoMotorConfiguration.
   *
   * @param device CAN Device representing the Talon Fx
   * @param servoMotorConfig The servo motor configuration to create the new TalonFXIO with
   */
  public MotorIOTalonFXSim(SimulatedMotorConfiguration<TalonFXConfiguration> simulatedMotorConfig) {
    super(simulatedMotorConfig.kRealConfiguration);

    this.config = simulatedMotorConfig;

    simState = talon.getSimState();
    simState.Orientation =
        MotorIOTalonFXSim.computeSimMotorOrientation(
            config.kRealConfiguration.getMotorConfig().MotorOutput.Inverted);

    setupMotorSim();

    lastUpdateTimestamp = Timer.getFPGATimestamp();
  }

  /** Setup the motor simulation and visualization */
  protected void setupMotorSim() {
    double gearRatio = config.kRealConfiguration.getGearRatio();
    var plant =
        LinearSystemId.createDCMotorSystem(
            config.kSimMotorConstants, config.kRealConfiguration.kMomentOfInertia, gearRatio);
    this.motorSim = new DCMotorSim(plant, config.kSimMotorConstants);

    double startAngleRotations =
        config.kRealConfiguration.getUnitsToMechanismRotations(config.kStartingRotationUnits);

    double minAngleRotations =
        config.kRealConfiguration.getUnitsToMechanismRotations(
            config.kRealConfiguration.kMinPositionUnits);

    double maxAngleRotations =
        config.kRealConfiguration.getUnitsToMechanismRotations(
            config.kRealConfiguration.kMaxPositionUnits);

    motorSim.setAngle(Units.rotationsToRadians(startAngleRotations));

    visualization =
        new SimulatedTalonFXVisualization(
            Units.rotationsToDegrees(maxAngleRotations),
            Units.rotationsToDegrees(minAngleRotations),
            Units.rotationsToDegrees(startAngleRotations));
  }

  /**
   * Create a new TalonFX Sim IO using a TalonFXConfiguration.
   *
   * @param device CAN Device representing the Talon Fx
   * @param motorConfig The basic motor configuration for this TalonFX
   */
  public MotorIOTalonFXSim(CANDeviceID device, TalonFXConfiguration motorConfig) {
    super(device, motorConfig);

    simState = talon.getSimState();
    simState.Orientation =
        MotorIOTalonFXSim.computeSimMotorOrientation(motorConfig.MotorOutput.Inverted);

    setupMotorSim();

    lastUpdateTimestamp = Timer.getFPGATimestamp();
  }

  /**
   * Create a new TalonFX Sim IO using a raw motor, this should pretty much ONLY be used on the
   * drivetrain
   *
   * <p>Doesnt utilize the built in motor sim
   *
   * @param motor The motor itself, the TalonFXIO is just a commonality wrapper
   */
  public MotorIOTalonFXSim(TalonFX motor) {
    super(motor);
    simState = talon.getSimState();

    lastUpdateTimestamp = Timer.getFPGATimestamp();
  }

  /** Update the state of the motor sim and inputs */
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

    double rawPosition = motorSim.getAngularPositionRad();

    double minAngleRad =
        Units.rotationsToRadians(
            config.kRealConfiguration.getUnitsToMechanismRotations(
                config.kRealConfiguration.kMinPositionUnits));
    double maxAngleRad =
        Units.rotationsToRadians(
            config.kRealConfiguration.getUnitsToMechanismRotations(
                config.kRealConfiguration.kMaxPositionUnits));

    double clampedPosition = MathUtil.clamp(rawPosition, minAngleRad, maxAngleRad);

    if (rawPosition != clampedPosition) {
      motorSim.setAngle(clampedPosition);
      double currentVelocity = motorSim.getAngularVelocityRadPerSec();
      if ((rawPosition < minAngleRad && currentVelocity < 0)
          || (rawPosition > maxAngleRad && currentVelocity > 0)) {
        motorSim.setAngularVelocity(0);
      }
    }

    inputs.SimPosUnits =
        config.kRealConfiguration.getMechanismRotationsToUnits(
            Units.radiansToRotations(motorSim.getAngularPositionRad()));
    inputs.SimVelocityUnits =
        config.kRealConfiguration.getMechanismRotationsToUnits(
            Units.radiansToRotations(motorSim.getAngularVelocityRadPerSec()));
    inputs.SimAccelerationUnits =
        config.kRealConfiguration.getMechanismRotationsToUnits(
            Units.radiansToRotations(motorSim.getAngularAccelerationRadPerSecSq()));

    inputs.RotorPosition = config.kRealConfiguration.getUnitsToRotorRotations(inputs.SimPosUnits);
    inputs.RotorVelocity =
        config.kRealConfiguration.getUnitsToRotorRotations(inputs.SimVelocityUnits);
    inputs.RotorAcceleration =
        config.kRealConfiguration.getUnitsToRotorRotations(inputs.SimAccelerationUnits);

    simState.setRotorAcceleration(inputs.RotorAcceleration);
    simState.setRawRotorPosition(inputs.RotorPosition);
    simState.setRotorVelocity(inputs.RotorVelocity);

    visualization.updateAngle(inputs.SimPosUnits);
  }

  public void logSim(String standardPrefix, String inputPrefix) {
    Logger.recordOutput(standardPrefix + "/Simulation/SupplyVoltage", inputs.SupplyVoltage);
    Logger.recordOutput(standardPrefix + "/Simulation/Voltage", inputs.SimVoltage);
    Logger.recordOutput(standardPrefix + "/Simulation/PositionUnits", inputs.SimPosUnits);
    Logger.recordOutput(
        standardPrefix + "/Simulation/VelocityUnitsPerSec", inputs.SimVelocityUnits);
    Logger.recordOutput(standardPrefix + "/Simulation/RotorPosition", inputs.RotorPosition);
    Logger.recordOutput(standardPrefix + "/Simulation/RotorVelocity", inputs.RotorVelocity);
    Logger.recordOutput(standardPrefix + "/Simulation/Mechanism2d", visualization.getMech2d());
  }

  public TalonFXSimState getCTRESimState() {
    return simState;
  }

  /** Forcibly set the simulated velocity to the given velocity. */
  public void forceSetMotorVelocity(double velocityUnitsPerSecond) {
    motorSim.setAngularVelocity(
        Units.rotationsToRadians(
            config.kRealConfiguration.getUnitsToRotorRotations(velocityUnitsPerSecond)));
  }

  public SimulatedTalonFXState getSimState() {
    return this.inputs;
  }

  /**
   * Forcibly updates the internal simulation state of the motor using measured encoder values and
   * battery voltage, then calculates and returns the corresponding motor voltage measure.
   *
   * <p>This method is intended for use within a simulation environment to synchronize the simulated
   * motor model's state (position, velocity, supply voltage) with the external measurements being
   * fed to it (e.g., from an encoder or another physics model).
   *
   * @param mechanismAngle The current angular position of the mechanism (e.g., arm, wheel) that the
   *     motor is driving. Not directly used in the current implementation.
   * @param mechanismVelocity The current angular velocity of the mechanism. Not directly used in
   *     the current implementation.
   * @param encoderAngle The latest angular position reading from the motor's encoder. This is used
   *     to set the motor's raw rotor position in the simulation.
   * @param encoderVelocity The latest angular velocity reading from the motor's encoder. This is
   *     used to set the motor's rotor velocity in the simulation.
   * @return A {@link Voltage} object representing the calculated motor voltage measure that should
   *     be applied to the motor for the next simulation step, based on the updated state.
   */
  @Override
  public Voltage updateControlSignal(
      Angle mechanismAngle,
      AngularVelocity mechanismVelocity,
      Angle encoderAngle,
      AngularVelocity encoderVelocity) {
    simState.setRawRotorPosition(encoderAngle);
    simState.setRotorVelocity(encoderVelocity);
    simState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());

    return simState.getMotorVoltageMeasure();
  }

  /**
   * Compute the proper chassis orientation given some motor inverted value
   *
   * @param value The inverted direction of the motor
   * @return The chassis centric orientation of the motor
   */
  private static ChassisReference computeSimMotorOrientation(InvertedValue value) {
    return (value == InvertedValue.Clockwise_Positive)
        ? ChassisReference.Clockwise_Positive
        : ChassisReference.CounterClockwise_Positive;
  }
}
