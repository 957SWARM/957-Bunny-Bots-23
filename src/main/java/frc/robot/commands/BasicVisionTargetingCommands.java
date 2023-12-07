package frc.robot.commands;

import com.team957.lib.controllers.feedback.PID;
import com.team957.lib.controllers.feedback.PID.PIDConstants;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionTargetingConstants;
import frc.robot.commands.drivetrain.ChassisControlCommand;
import frc.robot.subsystems.DriveSubsystem;
import java.util.function.DoubleSupplier;

public class BasicVisionTargetingCommands {

    public static double clamp(double in) {
        if (in > 0.25) {
            return 0.25;
        } else if (in < -0.25) {
            return -0.25;
        }
        return in;
    }

    public static Command getBasicVisionTargeting(
            DriveSubsystem driveSubsystem, DoubleSupplier txSupplier) {
        PIDConstants pidConstants =
                new PIDConstants(
                        VisionTargetingConstants.TARGETING_KP,
                        VisionTargetingConstants.TARGETING_KI,
                        VisionTargetingConstants.TARGETING_KD);

        final PID pid = new PID(pidConstants, 0, false);

        return new ChassisControlCommand(
                driveSubsystem,
                () -> 0,
                () -> 0,
                // () -> clamp(pid.calculate(Units.degreesToRadians(txSupplier.getAsDouble()))));
                () -> 0.1);

        // TODO: run closed-loop off of gyro, not LL
    }
}
