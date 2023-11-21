package frc.robot.input;

public interface DriverInput {

    public double swerveX();

    public double swerveY();

    public boolean swerveFieldRelative();

    public boolean zeroGyro();

    public double swerveRot();

    public boolean toggleGrabber();

    public boolean cancel();

    public boolean shoot();

    public boolean eject();

    public boolean intake();

    public boolean visionTargeting();

    public boolean increaseBallCount();

    public boolean decreaseBallCount();
}
