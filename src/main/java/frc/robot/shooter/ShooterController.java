package frc.robot.shooter;

import com.team957.lib.controllers.feedback.PID;
import com.team957.lib.util.DeltaTimeUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;

public class ShooterController extends CommandBase {
    private final Shooter shooter;

    private final DoubleSupplier setpointRadiansPerSecond;

    private final boolean useFeedback;
    private final boolean useFeedforward;
    private final boolean useProfiling;

    private final DeltaTimeUtil dtUtil;

    private final PID feedback =
            new PID(
                    Constants.ShooterConstants.FEEDBACK_CONSTANTS,
                    Constants.ShooterConstants.FEEDBACK_INTEGRATION_WINDOW,
                    0);

    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 0, 0);

    public ShooterController(
            Shooter shooter,
            DoubleSupplier setpointRadiansPerSecond,
            boolean useFeedback,
            boolean useFeedforward,
            boolean useProfiling) {
        addRequirements(shooter);

        this.setpointRadiansPerSecond = setpointRadiansPerSecond;

        this.shooter = shooter;

        this.useFeedback = useFeedback;
        this.useFeedforward = useFeedforward;
        this.useProfiling = useProfiling;

        dtUtil = new DeltaTimeUtil();
    }

    public ShooterController(Shooter shooter, DoubleSupplier setpointRadiansPerSecond) {
        this(shooter, setpointRadiansPerSecond, true, true, true);
    }

    @Override
    public void execute() {
        double dt = dtUtil.getTimeSecondsSinceLastCall();

        double profiledSetpoint;

        double setpointVelocity;

        if (useProfiling) {
            TrapezoidProfile profile =
                    new TrapezoidProfile(
                            Constants.ShooterConstants.MOTION_PROFILE_CONSTRAINTS,
                            new TrapezoidProfile.State(
                                    shooter.getFlywheelVelocityRadsPerS(),
                                    shooter.getFlywheelAccelerationRadsPerSSquard()),
                            new TrapezoidProfile.State(setpointRadiansPerSecond.getAsDouble(), 0));

            TrapezoidProfile.State state = profile.calculate(dt);

            profiledSetpoint = state.position; // actually velocity
            setpointVelocity = state.velocity; // actually acceleration
        } else {
            profiledSetpoint = setpointRadiansPerSecond.getAsDouble();

            setpointVelocity = 0;
        }

        double controlEffort = 0;

        if (useFeedback) {
            feedback.setSetpoint(profiledSetpoint);
            controlEffort += feedback.calculate(shooter.getFlywheelVelocityRadsPerS(), dt);
        }

        if (useFeedforward) {
            controlEffort += feedforward.calculate(profiledSetpoint, setpointVelocity);
        }

        shooter.setFlywheelVoltage(controlEffort);
    }
}
