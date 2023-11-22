package frc.robot.commands.drivetrain;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.team957.lib.controllers.feedback.PID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.PathPlannerUtil;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class PathFollowCommands {
    public Command getPathFollowCommand(
            DriveSubsystem drive, Supplier<Pose2d> localization, PathPlannerTrajectory path) {
        final PID xFeedback = new PID(Constants.PathFollowingConstants.LINEAR_CASCADE_CONSTANTS, 0);
        final PID yFeedback = new PID(Constants.PathFollowingConstants.LINEAR_CASCADE_CONSTANTS, 0);
        final PID thetaFeedback =
                new PID(Constants.PathFollowingConstants.ANGULAR_CASCADE_CONSTANTS, 0, true);

        final Timer timer = new Timer();
        timer.stop();
        timer.reset();

        final DoubleSupplier xVelocitySetpoint =
                () -> {
                    PathPlannerState state = PathPlannerUtil.sampleFullState(timer.get(), path);

                    double feedforward =
                            state.velocityMetersPerSecond * state.poseMeters.getRotation().getCos();
                    // state.pose.rotation cooresponds to the velocity vector

                    xFeedback.setSetpoint(state.poseMeters.getX());
                    double feedback = xFeedback.calculate(localization.get().getX());

                    return feedback + feedforward;
                };

        final DoubleSupplier yVelocitySetpoint =
                () -> {
                    PathPlannerState state = PathPlannerUtil.sampleFullState(timer.get(), path);

                    double feedforward =
                            state.velocityMetersPerSecond * state.poseMeters.getRotation().getSin();
                    // state.pose.rotation cooresponds to the velocity vector

                    yFeedback.setSetpoint(state.poseMeters.getY());
                    double feedback = yFeedback.calculate(localization.get().getY());

                    return feedback + feedforward;
                };

        final DoubleSupplier angularVelocitySetpoint =
                () -> {
                    PathPlannerState state = PathPlannerUtil.sampleFullState(timer.get(), path);

                    double feedforward = state.angularVelocityRadPerSec;

                    thetaFeedback.setSetpoint(state.holonomicRotation.getRadians());
                    double feedback = thetaFeedback.calculate(localization.get().getY());

                    return feedback + feedforward;
                };

        return new ChassisControlCommand(
                        drive, xVelocitySetpoint, yVelocitySetpoint, angularVelocitySetpoint)
                .alongWith(Commands.runOnce(timer::start));
    }
}
