package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.VisionTargetingConstants;
import frc.robot.microsystems.Limelight;
import frc.robot.subsystems.DriveSubsystem;
import java.util.function.DoubleSupplier;

public class BasicVisionTargetingCommand extends CommandBase {
    private final Limelight limelight = Limelight.getInstance();
    private final DriveSubsystem driveSubsystem;

    private final DoubleSupplier swerveX;
    private final DoubleSupplier swerveY;

    PIDController targetingPID =
            new PIDController(
                    VisionTargetingConstants.TARGETING_KP,
                    VisionTargetingConstants.TARGETING_KI,
                    VisionTargetingConstants.TARGETING_KD);

    public BasicVisionTargetingCommand(
            DriveSubsystem driveSubsystem, DoubleSupplier swerveX, DoubleSupplier swerveY) {
        this.driveSubsystem = driveSubsystem;
        this.swerveX = swerveX;
        this.swerveY = swerveY;
        addRequirements(driveSubsystem);
    }

    public void execute() {
        double tapeTx = limelight.getTx();
        // if (tapeTx > 0.5) {
        //     driveSubsystem.drive(
        //             swerveY.getAsDouble(),
        //             swerveX.getAsDouble(),
        //             targetingPID.calculate(tapeTx, 0),
        //             false,
        //             false);
        // }
    }
}
