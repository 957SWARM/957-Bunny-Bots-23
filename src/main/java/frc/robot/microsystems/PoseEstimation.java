package frc.robot.microsystems;

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
    }

    public Pose2d getPoseEstimate() {
        return odometry.getPoseMeters();
    }
}
