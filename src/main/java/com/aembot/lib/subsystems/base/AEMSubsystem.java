package com.aembot.lib.subsystems.base;

import com.aembot.lib.core.logging.Loggable;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class AEMSubsystem extends SubsystemBase implements Loggable {
  // Name of the subsystem that is in use
  public final String subsystemName;

  /** Log prefix used for standard logs */
  protected final String logPrefixStandard;

  /** Log prefix used for input logs */
  protected final String logPrefixInput;

  /** Base level Subsystem that handles basic logging functionality */
  public AEMSubsystem(String name) {
    this.subsystemName = name;

    this.logPrefixStandard = "Subsystems/" + this.subsystemName;

    this.logPrefixInput = "Inputs/" + logPrefixStandard;
  }

  @Override
  public void updateLog() {
    this.updateLog(this.logPrefixStandard, this.logPrefixInput);
  }
}
