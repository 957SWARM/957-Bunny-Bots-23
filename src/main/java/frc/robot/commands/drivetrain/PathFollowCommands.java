package frc.robot.commands.drivetrain;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoControlFunction;
import com.choreo.lib.ChoreoTrajectory;
import com.choreo.lib.ChoreoTrajectoryState;
import com.team957.lib.telemetry.HighLevelLogger;
import com.team957.lib.telemetry.Logger;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import java.util.function.Supplier;

public class PathFollowCommands {
    private static final Logger<Pose2d> postSetpointLogger =
            new Logger<>(
                    HighLevelLogger.getInstance().getLog(),
                    "poseSetpoint",
                    "poseSetpoint",
                    true,
                    true);

    public static Command getPathFollowCommand(
            DriveSubsystem drive, Supplier<Pose2d> localization, ChoreoTrajectory path) {

        final Timer timer = new Timer();
        timer.stop();
        timer.reset();

        final PIDController xController =
                new PIDController(
                        Constants.PathFollowingConstants.LINEAR_KP,
                        Constants.PathFollowingConstants.LINEAR_KI,
                        Constants.PathFollowingConstants.LINEAR_KD);

        final PIDController yController =
                new PIDController(
                        Constants.PathFollowingConstants.LINEAR_KP,
                        Constants.PathFollowingConstants.LINEAR_KI,
                        Constants.PathFollowingConstants.LINEAR_KD);

        final PIDController angularController =
                new PIDController(
                        Constants.PathFollowingConstants.ROTATIONAL_KP,
                        Constants.PathFollowingConstants.ROTATIONAL_KI,
                        Constants.PathFollowingConstants.ROTATIONAL_KD);

        angularController.enableContinuousInput(-Math.PI, Math.PI);

        final ChoreoControlFunction choreoPlanner =
                Choreo.choreoSwerveController(xController, yController, angularController);

        final Runnable updateLogs =
                () -> {
                    ChoreoTrajectoryState state = path.sample(timer.get());

                    postSetpointLogger.update(
                            new Pose2d(state.x, state.y, Rotation2d.fromRadians(state.heading)));
                };

        return Commands.runOnce(timer::start)
                .andThen(
                        new ChassisControlCommand(
                                        drive,
                                        () ->
                                                choreoPlanner.apply(
                                                        localization.get(),
                                                        path.sample(timer.get())))
                                .alongWith(Commands.run(updateLogs)));

        // this method dedicated to AJ for the usb-c charger
    }
}
