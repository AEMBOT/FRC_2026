package com.aembot.lib.core.motors.io;

import com.aembot.lib.config.motors.factories.SimulatedMotorConfiguration;
import com.aembot.lib.core.can.CANDeviceID;
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
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

/** IO implementation for a simulated TalonFX */
public class MotorIOTalonFXSim extends MotorIOTalonFX implements SimulatedMotorController {

  class SimulatedTalonFXVisualisation {
    private final LoggedMechanism2d mech2d;
    private final LoggedMechanismRoot2d root;

    private final LoggedMechanismLigament2d maxAngleLig;

    private final LoggedMechanismLigament2d minAngleLig;

    private final LoggedMechanismLigament2d curAngleLig;

    public SimulatedTalonFXVisualisation(double maxAngle, double minAngle, double startingAngle) {
      this.mech2d = new LoggedMechanism2d(2, 2);
      this.mech2d.setBackgroundColor(new Color8Bit(Color.kBlack));
      this.root = mech2d.getRoot("root", 1, 1);

      this.maxAngleLig =
          new LoggedMechanismLigament2d(
              "MaxAngleLigament", 1, maxAngle, 2, new Color8Bit(Color.kGray));
      this.minAngleLig =
          new LoggedMechanismLigament2d(
              "MinAngleLigament", 1, minAngle, 2, new Color8Bit(Color.kGray));
      this.curAngleLig =
          new LoggedMechanismLigament2d(
              "CurAngleLigament", 1, startingAngle, 4, new Color8Bit(Color.kMediumAquamarine));

      this.root.append(maxAngleLig);
      this.root.append(minAngleLig);
      this.root.append(curAngleLig);
    }

    public void updateAngle(double newAngle) {
      this.curAngleLig.setAngle(newAngle);
    }

    public LoggedMechanism2d getMech2d() {
      return mech2d;
    }
  }

  class SimulatedTalonFXInputs {
    double SupplyVoltage;
    double SupplyCurrent;
    double SimVoltage;
    double SimPosDegrees;
    double SimVelocityDegrees;
    double RotorPosition;
    double RotorVelocity;
  }

  protected final TalonFXSimState simState;

  protected SimulatedTalonFXInputs inputs = new SimulatedTalonFXInputs();

  protected SimulatedMotorConfiguration<TalonFXConfiguration> config;

  protected DCMotorSim motorSim;

  protected double lastUpdateTimestamp = 0.0;

  protected SimulatedTalonFXVisualisation visualization;

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

  private void setupMotorSim() {
    double gearRatio = 360 / config.kRealConfiguration.kUnitToRotorRotationRatio;
    var plant =
        LinearSystemId.createDCMotorSystem(
            config.kSimMotorConstants, config.kRealConfiguration.kMomentOfInertia, gearRatio);
    this.motorSim = new DCMotorSim(plant, config.kSimMotorConstants);

    double startAngle = Units.degreesToRotations(config.kStartingRotationDegrees);
    motorSim.setAngle(Units.rotationsToRadians(startAngle));

    visualization =
        new SimulatedTalonFXVisualisation(
            config.kRealConfiguration.kMaxPositionUnits,
            config.kRealConfiguration.kMinPositionUnits,
            config.kStartingRotationDegrees);
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
   * @param motor The motor itself, the TalonFXIO is just a commonality wrapper
   */
  public MotorIOTalonFXSim(TalonFX motor) {
    super(motor);
    simState = talon.getSimState();

    setupMotorSim();

    lastUpdateTimestamp = Timer.getFPGATimestamp();
  }

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

    double rawPosition = motorSim.getAngularPositionRad();

    double minAngleRad = Units.degreesToRadians(config.kRealConfiguration.kMinPositionUnits);
    double maxAngleRad = Units.degreesToRadians(config.kRealConfiguration.kMaxPositionUnits);

    double clampedPosition = MathUtil.clamp(rawPosition, minAngleRad, maxAngleRad);

    if (rawPosition != clampedPosition) {
      motorSim.setAngle(clampedPosition);
      double currentVelocity = motorSim.getAngularVelocityRadPerSec();
      if ((rawPosition < minAngleRad && currentVelocity < 0)
          || (rawPosition > maxAngleRad && currentVelocity > 0)) {
        motorSim.setAngularVelocity(0);
      }
    }

    inputs.SimPosDegrees = Units.radiansToDegrees(motorSim.getAngularPositionRad());
    inputs.SimVelocityDegrees = Units.radiansToDegrees(motorSim.getAngularVelocityRadPerSec());

    inputs.RotorPosition = config.kRealConfiguration.getUnitsToRotorRotations(inputs.SimPosDegrees);
    inputs.RotorVelocity =
        config.kRealConfiguration.getUnitsToRotorRotations(inputs.SimVelocityDegrees);

    simState.setRawRotorPosition(inputs.RotorPosition);
    simState.setRotorVelocity(inputs.RotorVelocity);

    visualization.updateAngle(inputs.SimPosDegrees);
  }

  public void logSim(String standardPrefix, String inputPrefix) {
    Logger.recordOutput(standardPrefix + "/Simulation/SupplyVoltage", inputs.SupplyVoltage);
    Logger.recordOutput(standardPrefix + "/Simulation/SupplyCurrent", inputs.SupplyCurrent);
    Logger.recordOutput(standardPrefix + "/Simulation/Voltage", inputs.SimVoltage);
    Logger.recordOutput(standardPrefix + "/Simulation/PositionDegrees", inputs.SimPosDegrees);
    Logger.recordOutput(
        standardPrefix + "/Simulation/VelocityDegreesPerSec", inputs.SimVelocityDegrees);
    Logger.recordOutput(standardPrefix + "/Simulation/RotorPosition", inputs.RotorPosition);
    Logger.recordOutput(standardPrefix + "/Simulation/RotorVelocity", inputs.RotorVelocity);
    Logger.recordOutput(standardPrefix + "/Simulation/Mechaism2d", visualization.getMech2d());
  }

  public TalonFXSimState getSimState() {
    return simState;
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
