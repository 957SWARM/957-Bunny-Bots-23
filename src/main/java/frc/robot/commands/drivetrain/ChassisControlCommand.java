package frc.robot.commands.drivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import java.util.function.DoubleSupplier;

public class ChassisControlCommand extends ModuleControlCommand {
    /**
     * Constructs a ChassisControlCommand.
     *
     * @param drive The swerve drivetrain subsystem to depend upon.
     * @param chassisRelativeXVelocity Desired chassis-relative X velocity, in meters/second.
     * @param chassisRelativeYVelocity Desired chassis-relative Y velocity, in meters/second.
     * @param angularVelocity Desired CCW+ angular velocity, in radians/second.
     */
    public ChassisControlCommand(
            DriveSubsystem drive,
            DoubleSupplier chassisRelativeXVelocity,
            DoubleSupplier chassisRelativeYVelocity,
            DoubleSupplier angularVelocity) {
        super(
                drive,
                () -> {
                    SwerveModuleState[] states =
                            Constants.DriveConstants.KINEMATICS.toSwerveModuleStates(
                                    new ChassisSpeeds(
                                            chassisRelativeXVelocity.getAsDouble(),
                                            chassisRelativeYVelocity.getAsDouble(),
                                            angularVelocity.getAsDouble()));

                    SwerveDriveKinematics.desaturateWheelSpeeds(
                            states, Constants.DriveConstants.WHEEL_MAX_SPEED_METERS_PER_SECOND);

                    return new CombinedModuleSetpoints(states[1], states[2], states[3], states[0]);
                });
    }
}
