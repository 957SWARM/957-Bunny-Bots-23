package frc.robot.commands.drivetrain;

import com.team957.lib.controllers.feedback.PID;
import com.team957.lib.util.DeltaTimeUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveSubsystem.MaxSwerveModule;
import java.util.Set;
import java.util.function.Supplier;

/** Command for direct control of module states. */
public class ModuleControlCommand implements Command {
    private class ModuleContainer {
        final Supplier<SwerveModuleState> setpoint;

        final MaxSwerveModule module;

        final PID driveController = new PID(Constants.DriveConstants.DRIVE_FEEDBACK_CONSTANTS, 0);
        final PID steerController =
                new PID(Constants.DriveConstants.STEER_FEEDBACK_CONSTANTS, 0, true);

        ModuleContainer(Supplier<SwerveModuleState> setpoint, MaxSwerveModule module) {
            this.setpoint = setpoint;
            this.module = module;
        }
    }

    public static record CombinedModuleSetpoints(
            SwerveModuleState frontRight,
            SwerveModuleState frontLeft,
            SwerveModuleState backLeft,
            SwerveModuleState backRight) {}
    // this redundant record exists so that the command can be done in a single lambda, as is
    // the weird packing and unpacking of the lambda
    // doing it in multiple lambdas causes different modules to get setpoints that *slightly* clash.

    private final ModuleContainer frontRight;
    private final ModuleContainer frontLeft;
    private final ModuleContainer backLeft;
    private final ModuleContainer backRight;

    private final DeltaTimeUtil dtUtil;

    private final DriveSubsystem drive;

    public ModuleControlCommand(
            DriveSubsystem drive, Supplier<CombinedModuleSetpoints> moduleSetpoints) {
        this.frontRight =
                new ModuleContainer(() -> moduleSetpoints.get().frontRight, drive.frontRight);
        this.frontLeft =
                new ModuleContainer(() -> moduleSetpoints.get().frontLeft, drive.frontLeft);
        this.backLeft = new ModuleContainer(() -> moduleSetpoints.get().backLeft, drive.backLeft);
        this.backRight =
                new ModuleContainer(() -> moduleSetpoints.get().backRight, drive.backRight);

        this.dtUtil = new DeltaTimeUtil();

        this.drive = drive;
    }

    public Set<Subsystem> getRequirements() {
        return Set.of(drive);
    }

    @Override
    public void execute() {
        double dt = dtUtil.getTimeSecondsSinceLastCall();

        for (ModuleContainer module :
                new ModuleContainer[] {frontRight, frontLeft, backLeft, backRight}) {
            SwerveModuleState setpoint =
                    SwerveModuleState.optimize(
                            module.setpoint.get(),
                            new Rotation2d(module.module.getSteerPositionRadians()));

            module.driveController.setSetpoint(setpoint.speedMetersPerSecond);

            module.steerController.setSetpoint(setpoint.angle.getRadians());

            module.module.setDriveControlInput(
                    module.driveController.calculate(
                            module.module.getDriveVelocityMetersPerSecond(), dt));

            module.module.setSteerControlInput(
                    module.steerController.calculate(module.module.getSteerPositionRadians(), dt));
        }
    }
}
