package com.aembot.frc2026.config.robots;

import com.aembot.frc2026.config.subsystems.indexerKicker.IndexerKickerConfiguration;
import com.aembot.frc2026.config.subsystems.indexerSelector.IndexerSelectorConfiguration;
import com.aembot.frc2026.config.subsystems.spindexer.SpindexerConfiguration;
import com.aembot.lib.config.motors.MotorConfiguration;
import com.aembot.lib.config.motors.SimulatedMotorConfiguration;
import com.aembot.lib.config.sensors.timeOfFlight.TimeOfFlightConfiguration;
import com.aembot.lib.core.can.CANDeviceID;
import com.aembot.lib.core.can.CANDeviceID.CANDeviceType;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.system.plant.DCMotor;

public final class ProductionIndexerConfig {
  private final class GeneralConstants {
    public static final String COMPOUND_NAME = "IndexerCompound/";
  }

  /* ---- SPINDEXER ---- */
  private final class SpindexerConstants {
    static final String SUBSYSTEM_NAME = GeneralConstants.COMPOUND_NAME + "SpindexerSubsystem";

    static final double GEAR_RATIO = 3.0 / 1.0;

    static final int MOTOR_CAN_ID = 18;

    // constants copied from hood config
    static final Slot0Configs MOTOR_GAINS = new Slot0Configs().withKP(.1).withKV(.12);

    /** Target speed of the spindexer roller in RPM */
    static final double TARGET_SPEED_RPM = 200.0; // Copied from intake config

    /** Target acceleration of the spindexer roller in RPM^2. */
    static final double ACCELERATION_RPM = 400.0; // Copied from intake config

    /**
     * The amount of time it takes to transport a game piece from the spindexer to the selector.
     * Used in sim.
     */
    public static final double SECONDS_THRU_SPINDEXER = 0.1;

    /** Roughly the number of gamepieces the spindexer is able to hold. Used in sim */
    public static final int GAMEPIECE_CAPACITY = 34;

    public static SpindexerConfiguration makeSpindexerConfiguration(String busName) {
      MotorConfiguration<TalonFXConfiguration> motorConfig =
          new MotorConfiguration<TalonFXConfiguration>()
              .withMotorConfig(
                  new TalonFXConfiguration()
                      .withMotionMagic(
                          new MotionMagicConfigs()
                              .withMotionMagicCruiseVelocity((TARGET_SPEED_RPM / 60) * GEAR_RATIO)
                              .withMotionMagicAcceleration((ACCELERATION_RPM / 60) * GEAR_RATIO))
                      .withSlot0(MOTOR_GAINS))
              .withMomentOfInertia(0.01)
              .withCANDevice(
                  new CANDeviceID(
                      MOTOR_CAN_ID,
                      SUBSYSTEM_NAME + "Motor",
                      SUBSYSTEM_NAME,
                      CANDeviceID.CANDeviceType.TALON_FX,
                      busName))
              .withName(SUBSYSTEM_NAME + "Motor")
              .withUnitToMechanismRotationRatio(1) // Use RPM
              .withUnitToRotorRotationRatio(1 / GEAR_RATIO);

      SimulatedMotorConfiguration<TalonFXConfiguration> simMotorConfig =
          new SimulatedMotorConfiguration<TalonFXConfiguration>()
              .withRealConfiguration(motorConfig)
              .withStartingRotation(0)
              .withSimMotorConstants(DCMotor.getKrakenX60(1));

      return new SpindexerConfiguration(SUBSYSTEM_NAME)
          .withMotorConfig(motorConfig)
          .withSimMotorConfig(simMotorConfig)
          .withTargetSpeedRPM(TARGET_SPEED_RPM)
          .withGamePieceMoveTimeSeconds(SECONDS_THRU_SPINDEXER)
          .withGamePieceCapacity(GAMEPIECE_CAPACITY)
          .validate();
    }
  }

  /* ---- SELECTOR ---- */
  private final class SelectorConstants {
    static final String SUBSYSTEM_NAME = GeneralConstants.COMPOUND_NAME + "SelectorSubsystem";

    static final double GEAR_RATIO = 1.0 / 1.0;

    static final int MOTOR_CAN_ID = 19;
    static final int CANRANGE_CAN_ID = 21;

    // constants copied from hood config
    static final Slot0Configs MOTOR_GAINS = new Slot0Configs().withKP(.1).withKV(.12);

    /** Target speed of the spindexer roller in RPM */
    static final double TARGET_SPEED_RPM = 200.0; // Copied from intake config

    /** Target acceleration of the spindexer roller in RPM^2. */
    static final double ACCELERATION_RPM = 400.0; // Copied from intake config

    /** Threshold for the canrange to detect a game piece */
    static final double CANRANGE_THRESHOLD_METERS = 0.1;

    /**
     * The amount of time it takes to transport a game piece from the selector to the kicker. Used
     * in sim.
     */
    public static final double SECONDS_THRU_SELECTOR = 0.1;

    /** Roughly the number of gamepieces the selector is able to hold. Used in sim */
    public static final int GAMEPIECE_CAPACITY = 3;

    public static IndexerSelectorConfiguration makeSelectorConfiguration(String busName) {
      MotorConfiguration<TalonFXConfiguration> motorConfig =
          new MotorConfiguration<TalonFXConfiguration>()
              .withMotorConfig(
                  new TalonFXConfiguration()
                      .withMotionMagic(
                          new MotionMagicConfigs()
                              .withMotionMagicAcceleration((ACCELERATION_RPM / 60) * GEAR_RATIO))
                      .withSlot0(MOTOR_GAINS))
              .withMomentOfInertia(0.01)
              .withCANDevice(
                  new CANDeviceID(
                      MOTOR_CAN_ID,
                      SUBSYSTEM_NAME + "Motor",
                      SUBSYSTEM_NAME,
                      CANDeviceID.CANDeviceType.TALON_FX,
                      busName))
              .withName(SUBSYSTEM_NAME + "Motor")
              .withUnitToMechanismRotationRatio(1) // Use RPM
              .withUnitToRotorRotationRatio(GEAR_RATIO);

      SimulatedMotorConfiguration<TalonFXConfiguration> simMotorConfig =
          new SimulatedMotorConfiguration<TalonFXConfiguration>()
              .withRealConfiguration(motorConfig)
              .withStartingRotation(0)
              .withSimMotorConstants(DCMotor.getKrakenX60(1));

      TimeOfFlightConfiguration timeOfFlightConfig =
          new TimeOfFlightConfiguration("CANRange")
              .withCANDeviceID(
                  new CANDeviceID(
                      CANRANGE_CAN_ID,
                      SUBSYSTEM_NAME + "CANRange",
                      SUBSYSTEM_NAME,
                      CANDeviceType.CANRANGE,
                      busName))
              .withDetectionThresholdMeters(CANRANGE_THRESHOLD_METERS)
              .validate();

      return new IndexerSelectorConfiguration(SUBSYSTEM_NAME)
          .withMotorConfig(motorConfig)
          .withSimMotorConfig(simMotorConfig)
          .withTimeOfFlightConfig(timeOfFlightConfig)
          .withTargetSpeedRPM(TARGET_SPEED_RPM)
          .withGamePieceMoveTimeSeconds(SECONDS_THRU_SELECTOR)
          .withGamePieceCapacity(GAMEPIECE_CAPACITY)
          .validate();
    }
  }

  /* ---- KICKER ---- */
  private final class KickerConstants {
    static final String SUBSYSTEM_NAME = GeneralConstants.COMPOUND_NAME + "KickerSubsystem";

    static final double GEAR_RATIO = 1.0 / 1.0;

    static final int MOTOR_CAN_ID = 20;

    // constants copied from hood config
    static final Slot0Configs MOTOR_GAINS = new Slot0Configs().withKP(.1).withKV(.12);

    /** Target speed of the spindexer roller in RPM */
    static final double TARGET_SPEED_RPM = 200.0; // Copied from intake config

    /**
     * Speed of the kicker to resist movement of game pieces into the shooter in RPM.
     *
     * @see IndexerKickerConfiguration#kResistSpeedRPM
     */
    static final double RESIST_SPEED_RPM = -20.0;

    /** Target acceleration of the spindexer roller in RPM^2. */
    static final double ACCELERATION_RPM = 400.0; // Copied from intake config

    /**
     * The amount of time it takes to transport a game piece from the kicker to the shooter. Used in
     * sim.
     */
    public static final double SECONDS_THRU_KICKER = 0.05;

    /** Roughly the number of gamepieces the spindexer is able to hold. Used in sim */
    public static final int GAMEPIECE_CAPACITY = 1;

    public static IndexerKickerConfiguration makeKickerConfiguration(String busName) {
      MotorConfiguration<TalonFXConfiguration> motorConfig =
          new MotorConfiguration<TalonFXConfiguration>()
              .withMotorConfig(
                  new TalonFXConfiguration()
                      .withMotionMagic(
                          new MotionMagicConfigs()
                              .withMotionMagicAcceleration((ACCELERATION_RPM / 60) * GEAR_RATIO))
                      .withSlot0(MOTOR_GAINS))
              .withMomentOfInertia(0.01)
              .withCANDevice(
                  new CANDeviceID(
                      MOTOR_CAN_ID,
                      SUBSYSTEM_NAME + "Motor",
                      SUBSYSTEM_NAME,
                      CANDeviceID.CANDeviceType.TALON_FX,
                      busName))
              .withName(SUBSYSTEM_NAME + "Motor")
              .withUnitToMechanismRotationRatio(1) // Use RPM
              .withUnitToRotorRotationRatio(GEAR_RATIO);

      SimulatedMotorConfiguration<TalonFXConfiguration> simMotorConfig =
          new SimulatedMotorConfiguration<TalonFXConfiguration>()
              .withRealConfiguration(motorConfig)
              .withStartingRotation(0)
              .withSimMotorConstants(DCMotor.getKrakenX60(1));

      return new IndexerKickerConfiguration(SUBSYSTEM_NAME)
          .withMotorConfig(motorConfig)
          .withSimMotorConfig(simMotorConfig)
          .withTargetSpeedRPM(TARGET_SPEED_RPM)
          .withResistSpeedRPM(RESIST_SPEED_RPM)
          .withGamePieceMoveTimeSeconds(SECONDS_THRU_KICKER)
          .withGamePieceCapacity(GAMEPIECE_CAPACITY)
          .validate();
    }
  }

  public final SpindexerConfiguration spindexerConfiguration;
  public final IndexerSelectorConfiguration selectorConfiguration;
  public final IndexerKickerConfiguration kickerConfiguration;

  public ProductionIndexerConfig(String busName) {
    this.spindexerConfiguration = SpindexerConstants.makeSpindexerConfiguration(busName);
    this.selectorConfiguration = SelectorConstants.makeSelectorConfiguration(busName);
    this.kickerConfiguration = KickerConstants.makeKickerConfiguration(busName);
  }
}
