package frc.robot.commands.drivetrain;

import com.team957.lib.controllers.feedback.PID;
import com.team957.lib.util.DeltaTimeUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import java.util.function.DoubleSupplier;

/**
 * Command to control the swerve drivetrain.
 *
 * <p>This performs inverse kinematics to get the module states setpoints from the chassis
 * setpoints, and controls the modules to those setpoints.
 *
 * <p>This is not closed-loop chassis control and is not suitable for autonomous.
 */
public class ChassisControlCommand extends CommandBase {
    private final DriveSubsystem drive;

    private final DoubleSupplier angularVelocitySetpoint;
    private final DoubleSupplier xVelocitySetpoint;
    private final DoubleSupplier yVelocitySetpoint;

    private final DeltaTimeUtil dtUtil;

    private final PID frontLeftSteerFeedback =
            new PID(Constants.DriveConstants.STEER_FEEDBACK_CONSTANTS, 0);
    private final PID frontRightSteerFeedback =
            new PID(Constants.DriveConstants.STEER_FEEDBACK_CONSTANTS, 0);
    private final PID backRightSteerFeedback =
            new PID(Constants.DriveConstants.STEER_FEEDBACK_CONSTANTS, 0);
    private final PID backLeftSteerFeedback =
            new PID(Constants.DriveConstants.STEER_FEEDBACK_CONSTANTS, 0);

    private final PID frontLeftDriveFeedback =
            new PID(Constants.DriveConstants.DRIVE_FEEDBACK_CONSTANTS, 0);
    private final PID frontRightDriveFeedback =
            new PID(Constants.DriveConstants.DRIVE_FEEDBACK_CONSTANTS, 0);
    private final PID backRightDriveFeedback =
            new PID(Constants.DriveConstants.DRIVE_FEEDBACK_CONSTANTS, 0);
    private final PID backLeftDriveFeedback =
            new PID(Constants.DriveConstants.DRIVE_FEEDBACK_CONSTANTS, 0);

    private final SwerveDriveKinematics kinematics =
            new SwerveDriveKinematics(
                    Constants.DriveConstants.FRONT_LEFT_TRANSFORM,
                    Constants.DriveConstants.FRONT_RIGHT_TRANSFORM,
                    Constants.DriveConstants.BACK_RIGHT_TRANSFORM,
                    Constants.DriveConstants.BACK_LEFT_TRANSFORM);

    /**
     * Constructs a ChassisControlCommand.
     *
     * @param drive Drivetrain subsystem.
     * @param angularVelocitySetpoint Units in radians/second, CCW+.
     * @param xVelocitySetpoint Units in meters/second.
     * @param yVelocitySetpoint Units in meters/second.
     */
    public ChassisControlCommand(
            DriveSubsystem drive,
            DoubleSupplier angularVelocitySetpoint,
            DoubleSupplier xVelocitySetpoint,
            DoubleSupplier yVelocitySetpoint) {
        this.drive = drive;

        this.angularVelocitySetpoint = angularVelocitySetpoint;
        this.xVelocitySetpoint = xVelocitySetpoint;
        this.yVelocitySetpoint = yVelocitySetpoint;

        dtUtil = new DeltaTimeUtil();

        addRequirements(drive);
    }

    @Override
    public void execute() {
        SwerveModuleState[] states =
                kinematics.toSwerveModuleStates(
                        new ChassisSpeeds(
                                xVelocitySetpoint.getAsDouble(),
                                yVelocitySetpoint.getAsDouble(),
                                angularVelocitySetpoint.getAsDouble()));

        double dt = dtUtil.getTimeSecondsSinceLastCall();
    }
}
