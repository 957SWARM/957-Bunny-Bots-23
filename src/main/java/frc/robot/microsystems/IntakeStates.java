package frc.robot.microsystems;

import frc.robot.Constants.IntakeConstants;

public enum IntakeStates {
    EJECT(-IntakeConstants.RUNNING_VOLTAGE, "Eject"),
    INTAKE(IntakeConstants.RUNNING_VOLTAGE, "Intake"),
    IDLE(0, "Idle");

    private final double voltage;
    private final String identifier;

    private IntakeStates(double voltage, String identifier) {
        this.voltage = voltage;
        this.identifier = identifier;
    }

    public double voltage() {
        return voltage;
    }

    public String identifier() {
        return identifier;
    }
}
