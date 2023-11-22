package frc.robot.commands.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.drivetrain.ModuleControlCommand.CombinedModuleSetpoints;
import frc.robot.subsystems.DriveSubsystem;

public class LockDrivetrainCommand extends ParallelCommandGroup {
    public LockDrivetrainCommand(DriveSubsystem drive) {
        addCommands(
                new ModuleControlCommand(
                        drive,
                        () ->
                                new CombinedModuleSetpoints(
                                        new SwerveModuleState(0, new Rotation2d(-Math.PI / 4)), // \
                                        new SwerveModuleState(0, new Rotation2d(Math.PI / 4)), // /
                                        new SwerveModuleState(0, new Rotation2d(-Math.PI / 4)), // \
                                        new SwerveModuleState(
                                                0, new Rotation2d(Math.PI / 4)))), // /
                drive.setBrakeModeCommand(true));
    }
}
