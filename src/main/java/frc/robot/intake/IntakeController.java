package frc.robot.intake;

import com.team957.lib.math.filters.ExponentialMovingAverage;
import com.team957.lib.util.DeltaTimeUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import java.util.function.BooleanSupplier;

public class IntakeController extends CommandBase {
    private final Intake intake;

    private final BooleanSupplier runIntake;

    private final ExponentialMovingAverage currentFilter =
            new ExponentialMovingAverage(
                    Constants.IntakeConstants.CURRENT_FILTER_RESPONSE_CONSTANT);

    private final DeltaTimeUtil dtUtil;

    public IntakeController(Intake intake, BooleanSupplier runIntake) {
        addRequirements(intake);

        this.intake = intake;

        this.runIntake = runIntake;

        dtUtil = new DeltaTimeUtil();
    }

    @Override
    public void execute() {
        double dt = dtUtil.getTimeSecondsSinceLastCall();

        intake.setVoltage(runIntake.getAsBoolean() ? Constants.IntakeConstants.RUNNING_VOLTAGE : 0);

        currentFilter.calculate(intake.getCurrentAmps(), dt);
    }

    public boolean currentIsHigh() {
        return (currentFilter.getCurrentOutput()
                >= Constants.IntakeConstants.HIGH_CURRENT_THRESHOLD_AMPS);
    }
}
