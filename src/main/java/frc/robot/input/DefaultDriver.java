package frc.robot.input;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class DefaultDriver implements DriverInput {

    private final CommandXboxController xboxController;

    public DefaultDriver(int port) {

        xboxController = new CommandXboxController(0);
    }

    @Override
    public double swerveX() {
        return xboxController.getLeftX();
    }

    @Override
    public double swerveY() {
        return -xboxController.getLeftY();
    }

    @Override
    public boolean swerveFieldRelative() {
        return xboxController.start().getAsBoolean();
    }

    @Override
    public boolean zeroGyro() {
        return xboxController.back().getAsBoolean();
    }

    public double swerveRot() {
        return xboxController.getRightX();
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

    public boolean visionTargeting(){
        return xboxController.b().getAsBoolean();
    }

    public boolean increaseBallCount(){
        return xboxController.povUp().getAsBoolean();
    }

    public boolean decreaseBallCount(){
        return xboxController.povDown().getAsBoolean();
    }
}
