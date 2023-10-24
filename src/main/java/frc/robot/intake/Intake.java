package frc.robot.intake;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface Intake extends Subsystem {
    // methods TBD as we figure out breakbeams etc

    public void setVoltage(double voltage);

    public double getCurrentAmps();
}
