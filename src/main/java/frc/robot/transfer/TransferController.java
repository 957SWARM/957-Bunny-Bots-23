package frc.robot.transfer;

import com.team957.lib.math.filters.ExponentialMovingAverage;
import com.team957.lib.util.DeltaTimeUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import java.util.function.BooleanSupplier;

public class TransferController extends CommandBase {
    private final Transfer transfer;

    private final BooleanSupplier runControlWheel;

    private final DeltaTimeUtil dtUtil;

    private final ExponentialMovingAverage controlWheelCurrentFilter =
            new ExponentialMovingAverage(
                    Constants.TransferConstants.CONTROL_WHEEL_CURRENT_FILTER_RESPONSE_CONSTANT);

    public TransferController(Transfer transfer, BooleanSupplier runControlWheel) {
        addRequirements(transfer);

        this.transfer = transfer;

        this.runControlWheel = runControlWheel;

        dtUtil = new DeltaTimeUtil();
    }

    @Override
    public void execute() {
        double dt = dtUtil.getTimeSecondsSinceLastCall();

        transfer.setControlWheelVoltage(
                runControlWheel.getAsBoolean()
                        ? Constants.TransferConstants.CONTROL_WHEEL_RUNNING_VOLTAGE
                        : 0);

        controlWheelCurrentFilter.calculate(transfer.getControlWheelCurrentAmps(), dt);
    }

    public boolean controlWheelCurrentIsHigh() {
        return (controlWheelCurrentFilter.getCurrentOutput()
                >= Constants.TransferConstants.CONTROL_WHEEL_HIGH_CURRENT_THRESHOLD_AMPS);
    }
}
