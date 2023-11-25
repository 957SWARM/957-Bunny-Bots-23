package frc.robot.peripherals;

import frc.robot.Constants.IntakeConstants;

public enum IntakeStates {
    EJECT(-IntakeConstants.RUNNING_VOLTAGE),
    INTAKE(IntakeConstants.RUNNING_VOLTAGE),
    IDLE(0);

    private final double voltage;

    private IntakeStates(double voltage) {
        this.voltage = voltage;
    }

    public double voltage() {
        return voltage;
    }
}
