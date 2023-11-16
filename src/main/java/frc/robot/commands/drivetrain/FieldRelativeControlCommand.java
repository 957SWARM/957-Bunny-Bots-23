package frc.robot.commands.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import frc.robot.subsystems.DriveSubsystem;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class FieldRelativeControlCommand extends ProxyCommand {
    public FieldRelativeControlCommand(
            DriveSubsystem drive,
            Supplier<Rotation2d> rotation,
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
                                        .rotateBy(rotation.get().unaryMinus())
                                        .getX(),
                        () ->
                                new Translation2d(
                                                fieldRelativeXVelocity.getAsDouble(),
                                                fieldRelativeYVelocity.getAsDouble())
                                        .rotateBy(rotation.get().unaryMinus())
                                        .getY(),
                        angularVelocity));
    }
}
