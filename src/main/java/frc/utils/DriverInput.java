package frc.utils;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface DriverInput {

    public double swerveX();

    public double swerveY();

    public Trigger swerveFieldRelative();

    public Trigger zeroGyro();

    public double swerveRot();

    public Trigger toggleGrabber();

    public boolean eject();

    public boolean shoot();

    public boolean cancel();

    public boolean intake();
}
