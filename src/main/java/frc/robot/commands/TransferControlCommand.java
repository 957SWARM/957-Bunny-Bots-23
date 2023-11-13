package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TransferConstants;
import frc.robot.subsystems.TransferSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TransferControlCommand extends CommandBase {
    private final TransferSubsystem transfer;

    private final BooleanSupplier enableTransfer;
    private final DoubleSupplier currentShooterRPM;
    private final DoubleSupplier targetShooterRPM;

    public TransferControlCommand(
            TransferSubsystem transfer,
            BooleanSupplier enableTransfer,
            DoubleSupplier currentShooterRPM,
            DoubleSupplier targetShooterRPM) {
        this.transfer = transfer;

        this.enableTransfer = enableTransfer;
        this.currentShooterRPM = currentShooterRPM;
        this.targetShooterRPM = targetShooterRPM;

        addRequirements(transfer);
    }

    public void execute() {
        // TODO: isn't this out of the scope of this command? Ideally all the control logic is done
        // in some higher-level command, and this just recieves a boolean or (fwd/rev/stop)
        double difference =
                Math.abs(targetShooterRPM.getAsDouble() - currentShooterRPM.getAsDouble());
        // if the transfer should be on and the shooter is running at correct speed, turn on
        // transfer. Otherwise, turn it off.
        if (enableTransfer.getAsBoolean() && difference < TransferConstants.RPM_TOLERANCE) {
            transfer.on();
        } else {
            transfer.off();
        }
    }
}
