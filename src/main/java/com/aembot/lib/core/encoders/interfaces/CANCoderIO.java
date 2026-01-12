package com.aembot.lib.core.encoders.interfaces;

import com.aembot.lib.core.encoders.CANCoderInputs;

public interface CANCoderIO {
  /** Get then name of the CANCoder */
  public String getName();

  /**
   * Update the {@link CANCoderInputs} object passed in with the current inputs to the CANCoder
   *
   * @param inputs The inputs that are currently applied to the CANcoder
   * @return true on success false on failure
   */
  public boolean updateInputs(CANCoderInputs inputs);

  /**
   * Set the update rate of this CANCoder in Hz
   *
   * @param hz Rate at which to update the state of this CANcoder in Hz
   * @return true on success false on failure
   */
  public boolean setUpdateFrequency(double hz);
}
