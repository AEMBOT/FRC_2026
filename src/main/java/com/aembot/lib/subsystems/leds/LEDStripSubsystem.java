package com.aembot.lib.subsystems.leds;

import com.aembot.lib.core.logging.log_entries.LogEntry;
import com.aembot.lib.subsystems.base.AEMSubsystem;
import com.aembot.lib.subsystems.leds.interfaces.LEDStripIO;
import edu.wpi.first.wpilibj2.command.Command;

public class LEDStripSubsystem extends AEMSubsystem {
  public enum LEDPattern {
    OFF('0'),
    RED('r'),
    BLUE('b'),
    ORANGE('o'),
    GREEN('g'),
    YELLOW('y'),
    PURPLE('p'),
    AEMLIGHT('l'),
    AEMDARK('d'),
    /**
     * LED strips hue fade until next command sent. <br>
     * 🏳️‍🌈
     */
    GAY_FADE('f'),
    /** Shifting rainbow pattern */
    // TODO confirm that's actually what it does
    RAINBOW('w'),
    ;

    /** The character sent over serial switch to this pattern */
    public final char kCode;

    private LEDPattern(char code) {
      this.kCode = code;
    }
  }

  public enum LEDSpeed {
    /** Set the rate of the pattern to default */
    NORMAL('1'),
    /** Sets the rate of the pattern to be faster */
    FAST('2'),
    ;

    /** The character sent over serial switch to this pattern */
    public final char kCode;

    private LEDSpeed(char code) {
      this.kCode = code;
    }
  }

  /** Character to send over serial for a mementary sped-up flash of {@link LEDPattern#GAY_FADE} */
  private static final char HUE_FLASH_CODE = 's';

  /** The amount of time the hue flash takes. This is dependent on the arduino-side firmware. */
  private static final double HUE_FLASH_SECONDS = 0.3;

  // I think it's best to declare LogEntries like this for the sake of visibility
  @SuppressWarnings("unused")
  private final LogEntry<LEDPattern> kPatternLog;

  @SuppressWarnings("unused")
  private final LogEntry<LEDSpeed> kSpeedLog;

  private final LogEntry<Character> kSerialOutLog;

  private final LEDStripIO kIO;

  private LEDPattern currentPattern = LEDPattern.AEMLIGHT;
  private LEDSpeed currentSpeed = LEDSpeed.NORMAL;

  public LEDStripSubsystem(String name, LEDStripIO io) {
    super(name);

    kPatternLog =
        new LogEntry<>(
            this.logPrefixStandard + "/State", LEDPattern.class, 1, this::getCurrentPattern);
    kSpeedLog =
        new LogEntry<>(this.logPrefixStandard + "/Speed", LEDSpeed.class, 1, this::getCurrentSpeed);

    kSerialOutLog = new LogEntry<>(this.logPrefixStandard + "/SerialOut", Character.class, 1);

    kIO = io;
  }

  public LEDPattern getCurrentPattern() {
    return this.currentPattern;
  }

  public LEDSpeed getCurrentSpeed() {
    return this.currentSpeed;
  }

  public void setPattern(LEDPattern pattern) {
    if (getCurrentPattern() != pattern) {
      kIO.sendCode(pattern.kCode);
      kSerialOutLog.pushValue(pattern.kCode);
      this.currentPattern = pattern;
    }
    kPatternLog.pushValue(pattern);
  }

  public void setSpeed(LEDSpeed speed) {
    if (getCurrentSpeed() != speed) {
      kIO.sendCode(speed.kCode);
      kSerialOutLog.pushValue(speed.kCode);
      this.currentSpeed = speed;
    }
    kSpeedLog.pushValue(speed);
  }

  public void hueFlash() {
    kIO.sendCode(HUE_FLASH_CODE);
    kSerialOutLog.pushValue(HUE_FLASH_CODE);
  }

  public Command patternAndSpeedCommand(LEDPattern pattern, LEDSpeed speed) {
    return runOnce(() -> this.setPattern(pattern))
        .andThen(runOnce(() -> this.setSpeed(speed)))
        .withName(pattern.name() + ":" + speed.name())
        .ignoringDisable(true);
  }

  public Command hueFlashCommand() {
    return runOnce(this::hueFlash)
        .withTimeout(HUE_FLASH_SECONDS)
        .withName("hueFlashCommand")
        .ignoringDisable(true);
  }

  @Override
  public void updateLog(String standardPrefix, String inputPrefix) {}
}

// end
// hope you liked the code, make sure to like and subscribe!
