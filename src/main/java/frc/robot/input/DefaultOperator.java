package frc.robot.input;

import edu.wpi.first.wpilibj.XboxController;

public class DefaultOperator implements OperatorInput {

    private final XboxController xboxController;

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
