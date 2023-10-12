package frc.robot.subsystems.shooter;

public interface Shooter {
    public void setFlywheelVoltage(double volts);

    public double getFlywheelCurrent();

    public double getFlywheelVelocityRadsPerS();

    public double getFlywheelAccelerationRadsPerSSquard();

    // this needs to be called every loop in order for this to function
    public void periodic();
}
