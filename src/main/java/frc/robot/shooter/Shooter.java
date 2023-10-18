package frc.robot.shooter;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface Shooter extends Subsystem {
    public void setFlywheelVoltage(double volts);

    public double getFlywheelCurrent();

    public double getFlywheelVelocityRadsPerS();

    public double getFlywheelAccelerationRadsPerSSquard();
}
