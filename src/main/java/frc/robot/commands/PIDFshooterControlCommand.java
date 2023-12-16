package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;
import java.util.function.DoubleSupplier;

// Shooter Command
public class PIDFshooterControlCommand extends CommandBase {
    // private final ShooterSubsystem shooter;
    private final ShooterSubsystem shooter;
    private final DoubleSupplier targetRPM;
    private final PIDController pid;
    private final SimpleMotorFeedforward feedforward =
            new SimpleMotorFeedforward(
                    ShooterConstants.FEEDFORWARD_KS_VOLT,
                    ShooterConstants.FEEDFORWARD_KV_VOLT_SECONDS_PER_METER,
                    ShooterConstants.FEEDFORWARD_KA_VOLT_SECONDS_SQUARED_PER_RADIAN);

    public PIDFshooterControlCommand(ShooterSubsystem shooter, DoubleSupplier targetRPM) {
        this.shooter = shooter;
        this.targetRPM = targetRPM;
        pid =
                new PIDController(
                        ShooterConstants.PID_P, ShooterConstants.PID_I, ShooterConstants.PID_D);
        addRequirements(shooter);
    }

    public void execute() {
        double pidfOutput =
                pid.calculate(shooter.getRPM(), targetRPM.getAsDouble())
                        + feedforward.calculate(targetRPM.getAsDouble());
        shooter.setVoltage(pidfOutput);
    }
}
