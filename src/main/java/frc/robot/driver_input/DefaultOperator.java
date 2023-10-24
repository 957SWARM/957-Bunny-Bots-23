package frc.robot.driver_input;

import edu.wpi.first.wpilibj.XboxController;

public class DefaultOperator implements OperatorInput {

    XboxController xboxController;

    public DefaultOperator(int port) {
        xboxController = new XboxController(0);
    }

    public boolean limelightAutoShot() {
        return xboxController.getAButton();
    }

    public boolean immediateShot() {
        return xboxController.getBButton();
    }
}
