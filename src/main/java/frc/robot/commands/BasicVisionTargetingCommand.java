package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.commands.drivetrain.ChassisControlCommand;
import frc.robot.subsystems.DriveSubsystem;
import java.util.function.DoubleSupplier;

public class BasicVisionTargetingCommand extends ChassisControlCommand {
    public BasicVisionTargetingCommand(DriveSubsystem driveSubsystem, DoubleSupplier txSupplier) {
        super(
                driveSubsystem,
                () -> 0,
                () -> 0,
                () -> Constants.VisionTargetingConstants.TARGETING_KP * -txSupplier.getAsDouble());

        // TODO: run closed-loop off of gyro, not LL
    }
}
