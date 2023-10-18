package frc.robot.transfer;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface Transfer extends Subsystem {
    // methods TBD as we figure out breakbeams and sensors

    public void setControlWheelVoltage(double volts);

    public double getControlWheelCurrentAmps();
}
