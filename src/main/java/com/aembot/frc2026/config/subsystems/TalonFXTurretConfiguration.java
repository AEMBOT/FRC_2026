package com.aembot.frc2026.config.subsystems;

import com.aembot.lib.config.encoders.AEMCANCoderConfiguration;
import com.aembot.lib.config.motors.MotorConfiguration;
import com.aembot.lib.config.motors.SimulatedMotorConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.MathUtil;
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

  /** Magnet offset for CANcoder A */
  public double kCANcoderAOffset;

  /** Magnet offset for CANcoder B */
  public double kCANcoderBOffset;

  /** The origin pose of the turret for visualization in advantagescope. */
  public Pose3d kTurretOriginPose;

  public double startingRotation;

  /** How far we can be off in units for auto aim to still shoot */
  public double kAutoAimLeniance;

  /** Number of teeth on the big gear that both encoder gears mesh with */
  public int kBigGearTeeth = 100;

  /** Previous position for CRT continuity tracking */
  private Double previousPosition = null;

  /** Maximum encoder match error to consider a valid CRT solution */
  private static final double CRT_MATCH_THRESHOLD = 2.0;

  /** Maximum position jump to prefer continuity over best match */
  private static final double CONTINUITY_THRESHOLD_DEG = 50.0;

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
   * Get the absolute position of the mechanism from the encoder positions using CRT.
   *
   * <p>Uses the Chinese Remainder Theorem with two coprime gear teeth counts to determine absolute
   * position within a unique range of (teethA * teethB / bigGearTeeth) rotations.
   *
   * @param rawCANcoderAPos position in rotations of CANcoder A (after offset subtraction)
   * @param rawCANcoderBPos position in rotations of CANcoder B (after offset subtraction)
   * @return The absolute position of the mechanism in degrees
   */
  public double getMechanismRotationsFromEncoders(double rawCANcoderAPos, double rawCANcoderBPos) {
    // Wrap inputs to [0, 1) to handle negative values from offset subtraction
    double teethA = MathUtil.inputModulus(rawCANcoderAPos, 0.0, 1.0) * kCANcoderAGearTeeth;
    double teethB = MathUtil.inputModulus(rawCANcoderBPos, 0.0, 1.0) * kCANcoderBGearTeeth;

    int crtRange = kCANcoderAGearTeeth * kCANcoderBGearTeeth;

    double bestPosition = 0;
    double bestScore = Double.MAX_VALUE;
    double closestToPrev = 0;
    double closestToPrevDist = Double.MAX_VALUE;

    // Search all possible CRT positions
    for (int i = 0; i < kCANcoderBGearTeeth; i++) {
      double testPos = teethA + i * kCANcoderAGearTeeth;
      if (testPos >= crtRange) {
        testPos -= crtRange;
      }

      // Score: how well does encoder B match at this position?
      double expectedB = testPos % kCANcoderBGearTeeth;
      double errorB =
          Math.min(
              Math.abs(teethB - expectedB), kCANcoderBGearTeeth - Math.abs(teethB - expectedB));

      double position = (testPos / kBigGearTeeth) * 360.0 + startingRotation;

      if (errorB < bestScore) {
        bestScore = errorB;
        bestPosition = position;
      }

      // Track position closest to previous (for continuity)
      if (errorB < CRT_MATCH_THRESHOLD && previousPosition != null) {
        double dist = Math.abs(position - previousPosition);
        if (dist < closestToPrevDist) {
          closestToPrevDist = dist;
          closestToPrev = position;
        }
      }
    }

    // Prefer continuity if we have a valid match close to previous position
    double result = (closestToPrevDist < CONTINUITY_THRESHOLD_DEG) ? closestToPrev : bestPosition;

    previousPosition = result;
    return result;
  }

  /** Reset CRT continuity tracking (call on robot enable or turret homing) */
  public void resetCRTContinuity() {
    previousPosition = null;
  }
}
