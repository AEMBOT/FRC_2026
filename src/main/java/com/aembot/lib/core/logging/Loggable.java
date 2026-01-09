package com.aembot.lib.core.logging;

/** Interface providing helper methods for logging w/ prefixes */
public interface Loggable {
  /** By default, calls {@code updateLog("", "")}. Override for default prefixes. */
  public default void updateLog() {
    updateLog("", "");
  }

  /**
   * Log data from this object
   *
   * @param standardPrefix Log prefix for standard logs. Ex: "Subsystems/elevator"
   * @param inputPrefix Log prefix for IO logs. Ex: "RealOutputs/Subsystems/elevator"
   */
  public abstract void updateLog(String standardPrefix, String inputPrefix);
}
