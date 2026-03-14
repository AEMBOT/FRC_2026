package com.aembot.frc2026.config.subsystems;

import com.aembot.lib.config.encoders.AEMCANCoderConfiguration;
import com.aembot.lib.config.motors.MotorConfiguration;
import com.aembot.lib.config.motors.SimulatedMotorConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.geometry.Pose3d;

public class TalonFXTurretConfiguration {

  /** Name of the turret subsystem */
  public final String kName;

  /** Configuration of the real motor */
  public MotorConfiguration<TalonFXConfiguration> kRealMotorConfig;

  /** Configuration of the simulated motor */
  public SimulatedMotorConfiguration<TalonFXConfiguration> kSimMotorConfig;

  /** Configuration of CANcoder A */
  public AEMCANCoderConfiguration kCANcoderAConfig;

  /** Configuration of CANcoder B */
  public AEMCANCoderConfiguration kCANcoderBConfig;

  /** Number of teeth on the gear connecting the rotor to CANcoder A */
  public int kCANcoderAGearTeeth;

  /** Number of teeth on the gear connecting the rotor to CANcoder B */
  public int kCANcoderBGearTeeth;

  /** Number of teeth on the big mechanism gear next to both CANcoders */
  public int kMechanismTeeth;

  /** Magnet offset for CANcoder A */
  public double kCANcoderAOffset;

  /** Magnet offset for CANcoder B */
  public double kCANcoderBOffset;

  /** The origin pose of the turret for visualization in advantagescope. */
  public Pose3d kTurretOriginPose;

  public double startingRotation;

  /** How far we can be off in units for auto aim to still shoot */
  public double kAutoAimLeniance;

  /**
   * Create a new turret configuration
   *
   * @param name Name of the turret subsystem
   */
  public TalonFXTurretConfiguration(String name) {
    this.kName = name;
  }

  public TalonFXTurretConfiguration withRealMotorConfig(
      MotorConfiguration<TalonFXConfiguration> realMotorConfig) {
    this.kRealMotorConfig = realMotorConfig;
    return this;
  }

  public TalonFXTurretConfiguration withSimMotorConfig(
      SimulatedMotorConfiguration<TalonFXConfiguration> simMotorConfig) {
    this.kSimMotorConfig = simMotorConfig;
    return this;
  }

  public TalonFXTurretConfiguration withCANcoderAConfig(AEMCANCoderConfiguration CANcoderAConfig) {
    this.kCANcoderAConfig = CANcoderAConfig;
    return this;
  }

  public TalonFXTurretConfiguration withCANcoderBConfig(AEMCANCoderConfiguration CANcoderBConfig) {
    this.kCANcoderBConfig = CANcoderBConfig;
    return this;
  }

  public TalonFXTurretConfiguration withCANcoderAGearTeeth(int CANcoderAGearTeeth) {
    this.kCANcoderAGearTeeth = CANcoderAGearTeeth;
    return this;
  }

  public TalonFXTurretConfiguration withCANcoderBGearTeeth(int CANcoderBGearTeeth) {
    this.kCANcoderBGearTeeth = CANcoderBGearTeeth;
    return this;
  }

  public TalonFXTurretConfiguration withMechanismTeeth(int MechanismTeeth) {
    this.kMechanismTeeth = MechanismTeeth;
    return this;
  }

  public TalonFXTurretConfiguration withCANcoderAOffset(double CANcoderAOffset) {
    this.kCANcoderAOffset = CANcoderAOffset;
    return this;
  }

  public TalonFXTurretConfiguration withCANcoderBOffset(double CANcoderBOffset) {
    this.kCANcoderBOffset = CANcoderBOffset;
    return this;
  }

  /**
   * Set the origin pose of the turret for visualization in advantagescope.
   *
   * @return this {@link TalonFXTurretConfiguration} for chaining
   */
  public TalonFXTurretConfiguration withTurretOriginPose(Pose3d turretOriginPose) {
    this.kTurretOriginPose = turretOriginPose;
    return this;
  }

  public TalonFXTurretConfiguration withStartingRotation(double offet) {
    this.startingRotation = offet;
    return this;
  }

  /**
   * Set the amount of units that we can be off in order to still shoot
   *
   * <p>Counts both directions, so for example if this was 10, we could have a deviance of -10
   * through +10
   *
   * @return this {@link TalonFXTurretConfiguration} for chaining
   */
  public TalonFXTurretConfiguration withAutoAimLeniance(double autoAimLeniance) {
    this.kAutoAimLeniance = autoAimLeniance;
    return this;
  }

  /**
   * Get the absolute position of the mechanism from the encoder positions
   *
   * @param rawCANcoderAPos position in rotations of CANcoder A
   * @param rawCANcoderBPos position in rotations of CANcoder B
   * @param CANcoderAteeth number of teeth on CANcoder A
   * @param CANcoderBteeth number of teeth on CANcoder B
   * @param mechanismTeeth number of teeth on the bigger mechanism
   * @return The absolute position of the mechanism in rotations. If it fails, returns -1
   */
  public double getMechanismRotationsFromEncoders(
      double rawCANcoderAPos,
      double rawCANcoderBPos,
      int CANcoderAteeth,
      int CANcoderBteeth,
      int mechanismTeeth) { // now returns rots instead of degrees

    double currentEncoderATeeth =
        rawCANcoderAPos * CANcoderAteeth; // the current position of the encoder in terms of teeth
    double currentEncoderBTeeth = rawCANcoderBPos * CANcoderBteeth;

    for (double testPos = currentEncoderATeeth;
        testPos < CANcoderAteeth * CANcoderBteeth;
        testPos += CANcoderAteeth) {

      double encoderBTestPos = testPos % CANcoderBteeth;

      double diff = Math.abs(currentEncoderBTeeth - encoderBTestPos);
      double circularDiff = Math.min(diff, CANcoderBteeth - diff);

      if (circularDiff < 0.3) {
        return (testPos / mechanismTeeth);
      }
    }

    return -1;
  }
}
