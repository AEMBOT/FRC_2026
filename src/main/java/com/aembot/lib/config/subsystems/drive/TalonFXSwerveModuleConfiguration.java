package com.aembot.lib.config.subsystems.drive;

import com.aembot.lib.core.can.CANDeviceID;
import com.aembot.lib.core.motors.interfaces.MotorIO.NeutralMode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

/**
 * Implementation of the swerve module configuration when using 2 TalonFX motor controllers and one
 * CANCoder
 */
public class TalonFXSwerveModuleConfiguration
    extends SwerveModuleConfiguration<
        TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> {

  public TalonFXSwerveModuleConfiguration(
      String moduleName,
      CANDeviceID driveMotorID,
      CANDeviceID steerMotorID,
      CANDeviceID steerEncoderID) {
    super(moduleName, driveMotorID, steerMotorID, steerEncoderID);
  }

  public TalonFXSwerveModuleConfiguration(
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          constants,
      String moduleName,
      CANDeviceID driveMotorID,
      CANDeviceID steerMotorID,
      CANDeviceID steerEncoderID) {
    super(moduleName, driveMotorID, steerMotorID, steerEncoderID);
    this.ctreModuleConstants = constants;
    this.driveMotorConfiguration = constants.DriveMotorInitialConfigs;
    this.steerMotorConfiguration = constants.SteerMotorInitialConfigs;
  }

  @Override
  public SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      getCtreModuleConstants() {
    if (ctreModuleConstants == null) {
      double couplingGearRatio = (double) driveGearBox.getStage(0).getGearRatio();
      ctreModuleConstants =
          new SwerveModuleConstants<
                  TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
              .withDriveMotorId(driveMotorID.getDeviceID())
              .withSteerMotorId(steerMotorID.getDeviceID())
              .withEncoderId(steerEncoderID.getDeviceID())
              .withDriveMotorGearRatio(driveGearBox.getTotalRatio())
              .withSteerMotorGearRatio(steerGearBox.getTotalRatio())
              .withCouplingGearRatio(couplingGearRatio)
              .withDriveMotorInverted(driveMotorInverted)
              .withSteerMotorInverted(steerMotorInverted)
              .withEncoderInverted(steerEncoderInverted)
              .withEncoderOffset(steerEncoderOffsetRotations)
              .withLocationX(locationOffset.getX())
              .withLocationY(locationOffset.getY())
              .withDriveMotorClosedLoopOutput(SwerveModuleConstants.ClosedLoopOutputType.Voltage)
              .withSteerMotorClosedLoopOutput(SwerveModuleConstants.ClosedLoopOutputType.Voltage)
              .withDriveMotorGains(driveMotorGains)
              .withSteerMotorGains(steerMotorGains)
              .withDriveMotorType(SwerveModuleConstants.DriveMotorArrangement.TalonFX_Integrated)
              .withSteerMotorType(SwerveModuleConstants.SteerMotorArrangement.TalonFX_Integrated)
              .withDriveMotorInitialConfigs(driveMotorConfiguration)
              .withSteerMotorInitialConfigs(steerMotorConfiguration)
              .withEncoderInitialConfigs(steerEncoderConfiguration)
              .withDriveFrictionVoltage(driveFrictionVoltage)
              .withSteerFrictionVoltage(steerFrictionVoltage)
              .withDriveInertia(driveInertia)
              .withSteerInertia(steerInertia)
              .withSlipCurrent(driveMotorSlipCurrent)
              .withFeedbackSource(SwerveModuleConstants.SteerFeedbackType.FusedCANcoder)
              .withSpeedAt12Volts(maxRobotSpeedMeterPerSecond)
              .withWheelRadius(wheelRadiusM);
    }
    return ctreModuleConstants;
  }

  @Override
  public TalonFXConfiguration getDriveMotorConfiguration() {
    if (driveMotorConfiguration == null) {
      driveMotorConfiguration =
          new TalonFXConfiguration()
              .withCurrentLimits(
                  new CurrentLimitsConfigs()
                      .withSupplyCurrentLimitEnable(true)
                      .withStatorCurrentLimitEnable(true)
                      .withSupplyCurrentLimit(driveMotorSupplyCurrentLimit)
                      .withStatorCurrentLimit(driveMotorStatorCurrentLimit))
              .withMotorOutput(
                  new MotorOutputConfigs()
                      .withNeutralMode(
                          driveNeutralMode == NeutralMode.BRAKE
                              ? NeutralModeValue.Brake
                              : NeutralModeValue.Coast));
    }
    return driveMotorConfiguration;
  }

  @Override
  public TalonFXConfiguration getSteerMotorConfiguration() {
    if (steerMotorConfiguration == null) {
      steerMotorConfiguration =
          new TalonFXConfiguration()
              .withCurrentLimits(
                  new CurrentLimitsConfigs()
                      .withSupplyCurrentLimitEnable(true)
                      .withStatorCurrentLimitEnable(true)
                      .withSupplyCurrentLimit(steerMotorSupplyCurrentLimit)
                      .withStatorCurrentLimit(steerMotorStatorCurrentLimit))
              .withMotorOutput(
                  new MotorOutputConfigs()
                      .withNeutralMode(
                          steerNeutralMode == NeutralMode.BRAKE
                              ? NeutralModeValue.Brake
                              : NeutralModeValue.Coast));
    }
    return steerMotorConfiguration;
  }

  @Override
  public CANcoderConfiguration getSteerEncoderConfiguration() {
    if (steerEncoderConfiguration == null) {
      steerEncoderConfiguration = new CANcoderConfiguration();
    }
    return steerEncoderConfiguration;
  }
}
