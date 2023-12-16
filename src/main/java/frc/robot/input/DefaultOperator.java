package frc.robot.input;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class DefaultOperator implements OperatorInput {

    private final CommandXboxController xboxController;

    public DefaultOperator(int port) {
        xboxController = new CommandXboxController(port);
    }

    public boolean limelightAutoShot() {
        return xboxController.a().getAsBoolean();
    }

    public boolean immediateShot() {
        return xboxController.b().getAsBoolean();
    }

    public boolean increaseShootPower() {
        return xboxController.povUp().getAsBoolean();
    }

    public boolean decreaseShootPower() {
        return xboxController.povDown().getAsBoolean();
    }
}
