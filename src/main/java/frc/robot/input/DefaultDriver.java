package frc.robot.input;

import edu.wpi.first.wpilibj.XboxController;
import frc.utils.DriverInput;

public class DefaultDriver implements DriverInput {

    XboxController xboxController;

    public DefaultDriver(int port) {

        xboxController = new XboxController(0);
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
        return xboxController.getStartButton();
    }

    @Override
    public boolean zeroGyro() {
        return xboxController.getBackButton();
    }

    public double swerveRot() {
        return xboxController.getRightX();
    }
}
