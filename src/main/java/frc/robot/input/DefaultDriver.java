package frc.robot.input;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class DefaultDriver implements DriverInput {

    private static final double LIN_MAX_SPEED = 3;
    private static final double ROT_MAX_SPEED = 6;

    private final CommandXboxController xboxController;

    public DefaultDriver(int port) {

        xboxController = new CommandXboxController(0);
    }

    @Override
    public double swerveX() {
        return LIN_MAX_SPEED * xboxController.getLeftX();
    }

    @Override
    public double swerveY() {
        return LIN_MAX_SPEED * -xboxController.getLeftY();
    }

    @Override
    public boolean zeroGyro() {
        return xboxController.back().getAsBoolean();
    }

    public double swerveRot() {
        return -ROT_MAX_SPEED * xboxController.getRightX();
    }

    public boolean toggleGrabber() {
        return xboxController.leftBumper().getAsBoolean();
    }

    public boolean cancel() {
        return xboxController.y().getAsBoolean();
    }

    public boolean shoot() {
        return xboxController.b().getAsBoolean();
    }

    public boolean eject() {
        return xboxController.x().getAsBoolean();
    }

    public boolean intake() {
        return xboxController.a().getAsBoolean();
    }

    public boolean visionTargeting() {
        return xboxController.b().getAsBoolean();
    }

    public boolean increaseBallCount() {
        return xboxController.povUp().getAsBoolean();
    }

    public boolean decreaseBallCount() {
        return xboxController.povDown().getAsBoolean();
    }
}
