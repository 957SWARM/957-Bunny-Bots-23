package frc.robot.input;

public interface DriverInput {

    // in meters/sec
    public double swerveX();

    // in meters/sec
    public double swerveY();

    // on rising edge
    public boolean zeroGyro();

    // in radians/sec
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
