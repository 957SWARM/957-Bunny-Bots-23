package frc.robot.commands.drivetrain;

import com.team957.lib.controllers.feedback.PID;
import com.team957.lib.util.DeltaTimeUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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

        final MaxSwerveModule hw;

        final PID driveFeedback = new PID(Constants.DriveConstants.DRIVE_FEEDBACK_CONSTANTS, 0);
        final SimpleMotorFeedforward driveFeedforward =
                new SimpleMotorFeedforward(
                        Constants.DriveConstants.DRIVE_FEEDFORWARD_KS_VOLTS,
                        Constants.DriveConstants.DRIVE_FEEDFORWARD_KV_VOLT_SECONDS_PER_METER);

        final PID steerFeedback =
                new PID(Constants.DriveConstants.STEER_FEEDBACK_CONSTANTS, 0, true);
        // starting with no steer feedforward, could very well be added if the turning needs to be
        // snappier

        ModuleContainer(Supplier<SwerveModuleState> setpoint, MaxSwerveModule module) {
            this.setpoint = setpoint;
            this.hw = module;
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
                            new Rotation2d(module.hw.getSteerPositionRadians()));

            module.driveFeedback.setSetpoint(setpoint.speedMetersPerSecond);
            module.steerFeedback.setSetpoint(setpoint.angle.getRadians());

            double driveFeedback =
                    module.driveFeedback.calculate(module.hw.getDriveVelocityMetersPerSecond(), dt);
            double driveFeedforward =
                    module.driveFeedforward.calculate(setpoint.angle.getRadians());
            module.hw.setDriveControlInput(driveFeedback + driveFeedforward);

            double steerFeedback =
                    module.steerFeedback.calculate(module.hw.getSteerPositionRadians(), dt);
            module.hw.setSteerControlInput(steerFeedback);
        }
    }
}
