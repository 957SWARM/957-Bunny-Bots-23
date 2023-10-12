package frc.robot.subsystems.shooter;

public interface Shooter {
    public void setFlywheelVoltage(double volts);

    public double getFlywheelCurrent();

    public double getFlywheelVelocityRadsPerS();

    public void periodic();
}
