package frc.robot.peripherals;

import com.team957.lib.telemetry.HighLevelLogger;
import com.team957.lib.telemetry.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem.ModuleStates;
import java.util.function.Supplier;

public class PoseEstimation {
    private final Supplier<ModuleStates> swerveModuleStates;
    private final Supplier<Rotation2d> rotation;

    private final SwerveDriveOdometry odometry;

    private final Logger<Pose2d> poseEstimateLogger =
            new Logger<>(
                    this::getPoseEstimate,
                    HighLevelLogger.getInstance().getLog(),
                    "poseEstimateMeters",
                    "poseEstimation",
                    true,
                    true);

    public PoseEstimation(
            Supplier<ModuleStates> swerveModuleStates,
            Supplier<Rotation2d> rotation,
            Pose2d initialPose) {
        this.swerveModuleStates = swerveModuleStates;
        this.rotation = rotation;

        this.odometry =
                new SwerveDriveOdometry(
                        Constants.DriveConstants.KINEMATICS,
                        rotation.get(),
                        swerveModuleStates.get().asArray(),
                        initialPose);
    }

    public void update() {
        odometry.update(rotation.get(), swerveModuleStates.get().asArray());

        poseEstimateLogger.update();
    }

    public Pose2d getPoseEstimate() {
        return odometry.getPoseMeters();
    }

    public void overridePose(Pose2d newPose) {
        odometry.resetPosition(rotation.get(), swerveModuleStates.get().asArray(), newPose);
    }
}
