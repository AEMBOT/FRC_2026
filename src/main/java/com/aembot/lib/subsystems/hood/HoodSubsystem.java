package com.aembot.lib.subsystems.hood;

import com.aembot.lib.config.subsystems.hood.TalonFXHoodConfiguration;
import com.aembot.lib.subsystems.base.AEMSubsystem;
import com.aembot.lib.subsystems.base.MotorCANCoderSubsystem;
import com.aembot.lib.subsystems.base.MotorSubsystem;
import com.aembot.lib.subsystems.hood.io.TalonFXHoodHardwareIO;

public class HoodSubsystem extends MotorSubsystem {

    public HoodSubsystem(
        TalonFXHoodConfiguration config,
        TalonFXHoodHardwareIO io
    ) {
        super(config.kName, , io.getMotor(), config.kMotorConfig);
    }

    @Override
    public void periodic() {
        super.periodic();
    }

    @Override
    public void updateLog(String standardPrefix, String inputPrefix) {
        
    }
    
}
