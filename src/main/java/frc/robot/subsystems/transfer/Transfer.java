package frc.robot.subsystems.transfer;

public interface Transfer {
    // methods TBD as we figure out breakbeams and sensors

    public void setLiftVoltage(double volts);

    public double getLiftCurrentAmps();

    public void setControlWheelVoltage(double volts);

    public double getControlWheelCurrentAmps();
}
