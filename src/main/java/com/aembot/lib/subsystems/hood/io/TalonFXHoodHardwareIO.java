package com.aembot.lib.subsystems.hood.io;

import com.aembot.lib.config.subsystems.hood.TalonFXHoodConfiguration;
import com.aembot.lib.core.motors.io.MotorIOTalonFX;

public class TalonFXHoodHardwareIO implements HoodIO {

    private final MotorIOTalonFX motor;

    private final TalonFXHoodConfiguration config;

    public TalonFXHoodHardwareIO(
        TalonFXHoodConfiguration config
    ) {
        this.motor = new MotorIOTalonFX(config.kMotorConfig);
        this.config = config;
    }

    @Override
    public MotorIOTalonFX getMotor() { return motor; }
    
    @Override
    public void updateLog(String standardPrefix, String inputs) {

    }
}
