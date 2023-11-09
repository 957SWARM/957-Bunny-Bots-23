package frc.robot.commands.drivetrain;

import com.team957.lib.controllers.feedback.PID;
import com.team957.lib.util.DeltaTimeUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveSubsystem.MaxSwerveModule;
import java.util.function.DoubleSupplier;

/** Command for direct control of module states. */
public class ModuleControlCommand extends CommandBase {
    private class ModuleContainer {
        final ModuleSetpoint setpoint;

        final MaxSwerveModule module;

        final PID driveController = new PID(Constants.DriveConstants.DRIVE_FEEDBACK_CONSTANTS, 0);
        final PID steerController =
                new PID(Constants.DriveConstants.STEER_FEEDBACK_CONSTANTS, 0, true);

        ModuleContainer(ModuleSetpoint setpoint, MaxSwerveModule module) {
            this.setpoint = setpoint;
            this.module = module;
        }
    }

    public static record ModuleSetpoint(
            DoubleSupplier driveVelocityMetersPerSecond, DoubleSupplier steerPositionRadians) {}

    private final ModuleContainer frontRight;
    private final ModuleContainer frontLeft;
    private final ModuleContainer backLeft;
    private final ModuleContainer backRight;

    private final DeltaTimeUtil dtUtil;

    public ModuleControlCommand(
            DriveSubsystem drive,
            ModuleSetpoint frontRight,
            ModuleSetpoint frontLeft,
            ModuleSetpoint backLeft,
            ModuleSetpoint backRight) {
        this.frontRight = new ModuleContainer(frontRight, drive.frontRight);
        this.frontLeft = new ModuleContainer(frontLeft, drive.frontLeft);
        this.backLeft = new ModuleContainer(backLeft, drive.backLeft);
        this.backRight = new ModuleContainer(backRight, drive.backRight);

        this.dtUtil = new DeltaTimeUtil();

        addRequirements(drive);
    }

    @Override
    public void execute() {
        double dt = dtUtil.getTimeSecondsSinceLastCall();

        for (ModuleContainer module :
                new ModuleContainer[] {frontRight, frontLeft, backLeft, backRight}) {
            module.driveController.setSetpoint(
                    module.setpoint.driveVelocityMetersPerSecond.getAsDouble());

            module.steerController.setSetpoint(module.setpoint.steerPositionRadians.getAsDouble());

            module.module.setDriveControlInput(
                    module.driveController.calculate(
                            module.module.getDriveVelocityMetersPerSecond(), dt));

            module.module.setSteerControlInput(
                    module.steerController.calculate(module.module.getSteerPositionRadians(), dt));
        }
    }
}
