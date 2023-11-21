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
    public Trigger swerveFieldRelative() {
        return xboxController.start();
    }

    @Override
    public Trigger zeroGyro() {
        return xboxController.back();
    }

    public double swerveRot() {
        return xboxController.getRightX();
    }

    public Trigger toggleGrabber() {
        return xboxController.leftBumper();
    }

    public Trigger cancel() {
        return xboxController.y();
    }

    public Trigger shoot() {
        return xboxController.b();
    }

    public Trigger eject() {
        return xboxController.x();
    }

    public Trigger intake() {
        return xboxController.a();
    }

    public Trigger visionTargerting(){
        return xboxController.b();
    }
}
