package frc.robot.input;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface DriverInput {

    public double swerveX();

    public double swerveY();

    public Trigger swerveFieldRelative();

    public Trigger zeroGyro();

    public double swerveRot();

    public Trigger toggleGrabber();
}
