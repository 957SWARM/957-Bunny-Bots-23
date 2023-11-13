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
        return xboxController.a();
    }

    public boolean cancel() {
        return xboxController.back().getAsBoolean();
    }

    public boolean shoot() {
        return xboxController.back().getAsBoolean();
    }

    public boolean eject() {
        return xboxController.back().getAsBoolean();
    }

    public boolean intake() {
        return xboxController.back().getAsBoolean();
    }
}
