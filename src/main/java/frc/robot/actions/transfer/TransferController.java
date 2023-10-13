package frc.robot.actions.transfer;

import com.team957.lib.math.filters.ExponentialMovingAverage;
import com.team957.lib.util.DeltaTimeUtil;
import frc.robot.Constants;
import frc.robot.subsystems.transfer.Transfer;

public class TransferController {
    private final Transfer transfer;

    private final DeltaTimeUtil dtUtil;

    private final ExponentialMovingAverage liftCurrentFilter =
            new ExponentialMovingAverage(
                    Constants.TransferConstants.LIFT_CURRENT_FILTER_RESPONSE_CONSTANT);
    private final ExponentialMovingAverage controlWheelCurrentFilter =
            new ExponentialMovingAverage(
                    Constants.TransferConstants.CONTROL_WHEEL_CURRENT_FILTER_RESPONSE_CONSTANT);

    public TransferController(Transfer transfer) {
        this.transfer = transfer;

        dtUtil = new DeltaTimeUtil();
    }

    public void execute(boolean runLift, boolean runControlWheel) {
        double dt = dtUtil.getTimeSecondsSinceLastCall();

        transfer.setLiftVoltage(runLift ? Constants.TransferConstants.LIFT_RUNNING_VOLTAGE : 0);
        transfer.setControlWheelVoltage(
                runControlWheel ? Constants.TransferConstants.CONTROL_WHEEL_RUNNING_VOLTAGE : 0);

        liftCurrentFilter.calculate(transfer.getLiftCurrentAmps(), dt);
        controlWheelCurrentFilter.calculate(transfer.getControlWheelCurrentAmps(), dt);
    }

    public boolean liftCurrentIsHigh() {
        return (liftCurrentFilter.getCurrentOutput()
                >= Constants.TransferConstants.LIFT_HIGH_CURRENT_THRESHOLD_AMPS);
    }

    public boolean controlWheelCurrentIsHigh() {
        return (controlWheelCurrentFilter.getCurrentOutput()
                >= Constants.TransferConstants.CONTROL_WHEEL_HIGH_CURRENT_THRESHOLD_AMPS);
    }
}
