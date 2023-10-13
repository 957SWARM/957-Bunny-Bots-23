package frc.robot.actions.intake;

import com.team957.lib.math.filters.ExponentialMovingAverage;
import com.team957.lib.util.DeltaTimeUtil;
import frc.robot.Constants;
import frc.robot.subsystems.intake.Intake;

public class IntakeController {
    private final Intake intake;

    private final ExponentialMovingAverage currentFilter =
            new ExponentialMovingAverage(
                    Constants.IntakeConstants.CURRENT_FILTER_RESPONSE_CONSTANT);

    private final DeltaTimeUtil dtUtil;

    public IntakeController(Intake intake) {
        this.intake = intake;

        dtUtil = new DeltaTimeUtil();
    }

    public void execute(boolean run) {
        double dt = dtUtil.getTimeSecondsSinceLastCall();

        intake.setVoltage(run ? Constants.IntakeConstants.RUNNING_VOLTAGE : 0);

        currentFilter.calculate(intake.getCurrentAmps(), dt);
    }

    public boolean currentIsHigh() {
        return (currentFilter.getCurrentOutput()
                >= Constants.IntakeConstants.HIGH_CURRENT_THRESHOLD_AMPS);
    }
}
