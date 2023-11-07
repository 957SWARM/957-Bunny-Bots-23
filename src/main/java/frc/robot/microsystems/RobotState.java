package frc.robot.microsystems;

public enum RobotState {
    IDLE("Idle"),
    EJECT("Reversing Intake"),
    INTAKE("Intaking"),
    SHOOT("Firing");

    private final String string_id;

    private RobotState(String string_id) {
        this.string_id = string_id;
    }

    public String id() {
        return string_id;
    }
}
