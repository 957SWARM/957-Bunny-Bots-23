package frc.robot.commands.drivetrain;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import frc.robot.microsystems.IMU;
import frc.robot.subsystems.DriveSubsystem;
import java.util.function.DoubleSupplier;

public class FieldRelativeControlCommand extends ProxyCommand {
    public FieldRelativeControlCommand(
            DriveSubsystem drive,
            IMU imu,
            DoubleSupplier fieldRelativeXVelocity,
            DoubleSupplier fieldRelativeYVelocity,
            DoubleSupplier angularVelocity) {
        super(
                new ChassisControlCommand(
                        drive,
                        () ->
                                // take the setpoints and rotate them backwards by the robot's
                                // rotation
                                new Translation2d(
                                                fieldRelativeXVelocity.getAsDouble(),
                                                fieldRelativeYVelocity.getAsDouble())
                                        .rotateBy(imu.getCorrectedAngle().unaryMinus())
                                        .getX(),
                        () ->
                                new Translation2d(
                                                fieldRelativeXVelocity.getAsDouble(),
                                                fieldRelativeYVelocity.getAsDouble())
                                        .rotateBy(imu.getCorrectedAngle().unaryMinus())
                                        .getY(),
                        angularVelocity));
    }
}
