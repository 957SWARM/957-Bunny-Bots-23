package frc.robot.commands;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;
import java.util.function.DoubleSupplier;

// Shooter Command
public class ShooterControlCommand extends CommandBase {
    private final ShooterSubsystem shooter;
    private final DoubleSupplier targetRPM;
    private final BangBangController bb;

    public ShooterControlCommand(ShooterSubsystem shooter, DoubleSupplier targetRPM) {
        this.shooter = shooter;
        this.targetRPM = targetRPM;
        bb = new BangBangController();
        addRequirements(shooter);
    }

    public void execute() {
        double bbOutput = bb.calculate(shooter.getRPM(), targetRPM.getAsDouble());

        shooter.setVoltage(bbOutput * Constants.ShooterConstants.RUNNING_VOLTAGE);
    }
}
