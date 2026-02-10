package com.aembot.frc2026.subsystems.turret;

import com.aembot.lib.config.motors.MotorConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfiguration;

// takes motorSubsystem, motorConfig, adds two cancoders. will be used by TurretSubsystem.
// set some consants or something
// need motorconfiguration, cancoderconfig1, cancoderconfig2, CC1 teeth, CC2 teeth, CC1 ratio
// (encoder full turns to units?), CC2 ratio
// chinese remainder theorem. will get x mod 13 = y from encoder 1 and x mod 17 = y from encoder 2.
// need x mod 100.
// pseudocode for solving position
/*
i=0
EncoderApos = round 13*rawEncoderApos
EncoderBpos = round 17*rawEncoderBpos
while solved=false:
if (EncoderBpos + 17*i) mod 13 = EncoderApos:
RealEncoderVal = EncoderBpos + 17*i + (rawEncoderApos mod (1/13))*13 the last addition is to add back the precision lost by rounding to int
solved=true
else:
i++

*/

public class CRTTurretConfiguration<C> {
  // constants here

  public MotorConfiguration<C> kMotorConfiguration = new MotorConfiguration<C>();
  public CANcoderConfiguration kCANCoderAConf = new CANcoderConfiguration();
  public CANcoderConfiguration kCANCoderBConf = new CANcoderConfiguration();

  public int kEncoderBteeth = 17;
  public int kEncoderAteeth = 13;
  // ratio of cancoder rotations to output units
  public double kEncoderBRatio = 1.;
  public double kEncoderARatio = 1.;

  // set input "config" to constant and add it to part of CRTTurretConfiguration
  public CRTTurretConfiguration<C> withMotorConfig(MotorConfiguration<C> config) {
    kMotorConfiguration = config;
    return this;
  }

  public CRTTurretConfiguration<C> withCANCoderAConfig(CANcoderConfiguration config) {
    kCANCoderAConf = config;
    return this;
  }

  public CRTTurretConfiguration<C> withCANCoderBConfig(CANcoderConfiguration config) {
    kCANCoderBConf = config;
    return this;
  }

  public CRTTurretConfiguration<C> withEncoderAteeth(int teeth) {
    kEncoderAteeth = teeth;
    return this;
  }

  public CRTTurretConfiguration<C> withEncoderBteeth(int teeth) {
    kEncoderBteeth = teeth;
    return this;
  }

  public CRTTurretConfiguration<C> withEncoderARatio(double ratio) {
    kEncoderARatio = ratio;
    return this;
  }

  public CRTTurretConfiguration<C> withEncoderBRatio(double ratio) {
    kEncoderBRatio = ratio;
    return this;
  }

  public double solve(double rawEncoderAinput, double rawEncoderBinput) {

    int i = 0;
    int EncoderApos = (int) Math.round(kEncoderAteeth * rawEncoderAinput);
    int EncoderBpos = (int) Math.round(kEncoderBteeth * rawEncoderBinput);
    boolean solved = false;
    double RealEncoderVal = 0;

    while (!solved) {
      if ((EncoderBpos + kEncoderBteeth * i) % kEncoderAteeth
          == EncoderApos) { // For this to be most efficient, Encoder B should be the bigger one
        RealEncoderVal = EncoderBpos;
        RealEncoderVal = RealEncoderVal + kEncoderBteeth * i;
        RealEncoderVal =
            RealEncoderVal + (rawEncoderAinput % (1. / kEncoderAteeth)) * kEncoderAteeth;
        solved = true;

      } else {
        i++;
      }
      if (i > kEncoderAteeth) {
        return -1; // means something is wrong. TODO add actual try catch except or whatever here
      }
    }
    System.out.println(kEncoderAteeth);
    System.out.println(kEncoderBteeth);
    System.out.println(rawEncoderAinput);
    System.out.println(rawEncoderBinput);
    System.out.println(RealEncoderVal);

    return RealEncoderVal;
  }
}
