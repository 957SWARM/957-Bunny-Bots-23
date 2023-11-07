package frc.robot.commands;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;
import frc.utils.Constants;
import java.util.function.DoubleSupplier;

// Shooter Command
public class FlywheelControlCommand extends CommandBase {
    private final ShooterSubsystem shooter;
    private final DoubleSupplier targetRPM;
    private final BangBangController bb;

    public FlywheelControlCommand(ShooterSubsystem shooter, DoubleSupplier targetRPM) {
        this.shooter = shooter;
        this.targetRPM = targetRPM;
        bb = new BangBangController();
    }

    public void execute() {
        double bbOutput = bb.calculate(shooter.getRPM(), targetRPM.getAsDouble());

        shooter.setVoltage(bbOutput * Constants.ShooterConstants.RUNNING_VOLTAGE);
    }
}
