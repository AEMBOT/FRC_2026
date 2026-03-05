package com.aembot.lib.config.sensors.timeOfFlight;

import com.aembot.lib.core.can.CANDeviceID;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.FovParamsConfigs;
import com.ctre.phoenix6.signals.UpdateModeValue;

public class CANRangeTimeOfFlightConfiguration extends TimeOfFlightConfiguration {
  public CANrangeConfiguration kCTREConfig = new CANrangeConfiguration();

  public CANRangeTimeOfFlightConfiguration(String name) {
    super(name);
  }

  /**
   * Set the CTRE configuration to use for this TOF sensor.
   *
   * <p><strong>NOTE</strong>: This generally doesn't need to be called; the field is set by
   * default. Should only be used if you're using a config from somewhere else for whatever reason.
   * If you do call this, do <strong>not</strong> call any of the CANRangeConfiguration-specific
   * "with" methods before it, as they will be overriden by this method.
   *
   * @return this {@link CANRangeTimeOfFlightConfiguration} for chaining
   */
  public CANRangeTimeOfFlightConfiguration withCTREConfig(CANrangeConfiguration ctreConfig) {
    this.kCTREConfig = ctreConfig;
    return this;
  }

  /**
   * Set the FOV parameters config to use for this TOF sensor.
   *
   * @return this {@link CANRangeTimeOfFlightConfiguration} for chaining
   */
  public CANRangeTimeOfFlightConfiguration withFOVConfig(FovParamsConfigs fovConfig) {
    this.kCTREConfig.withFovParams(fovConfig);
    return this;
  }

  /**
   * Set the update mode of the CANrange. The CANrange supports short-range and long-range detection
   * at various update frequencies.
   *
   * @return this {@link CANRangeTimeOfFlightConfiguration} for chaining
   */
  public CANRangeTimeOfFlightConfiguration withUpdateMode(UpdateModeValue updateMode) {
    this.kCTREConfig.ToFParams.withUpdateMode(updateMode);
    return this;
  }

  /**
   * Set the rate at which the CANrange will take measurements. A lower frequency may provide more
   * stable readings but will reduce the data rate of the sensor.
   *
   * @return this {@link CANRangeTimeOfFlightConfiguration} for chaining
   */
  public CANRangeTimeOfFlightConfiguration withUpdateFrequencyHz(double frequencyHz) {
    this.kCTREConfig.ToFParams.withUpdateFrequency(frequencyHz);
    return this;
  }

  /* ---- OVERRIDES ---- */
  @Override
  public CANRangeTimeOfFlightConfiguration withCANDeviceID(CANDeviceID id) {
    super.withCANDeviceID(id);
    return this;
  }

  @Override
  public CANRangeTimeOfFlightConfiguration withDetectionThresholdMeters(double threshold) {
    super.withDetectionThresholdMeters(threshold);
    // we don't actually use CTRE's prox detection, but it's useful for diagnostics
    kCTREConfig.ProximityParams.withProximityThreshold(threshold);
    return this;
  }

  @Override
  public CANRangeTimeOfFlightConfiguration withDetectionHysteresisMeters(double hysteresis) {
    super.withDetectionHysteresisMeters(hysteresis);
    // we don't actually use CTRE's prox detection, but it's useful for diagnostics
    kCTREConfig.ProximityParams.withProximityHysteresis(hysteresis);
    return this;
  }

  @Override
  public CANRangeTimeOfFlightConfiguration validate() {
    super.validate();
    return this;
  }
}
