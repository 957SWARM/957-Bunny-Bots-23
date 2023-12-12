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
    private double difference = 0;
    boolean speedReached = false;

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
        difference = Math.abs(targetShooterRPM.getAsDouble() - currentShooterRPM.getAsDouble());
        System.out.println("Difference: " + difference);
        // if the transfer should be on and the shooter is running at correct speed, turn on
        // transfer. Otherwise, turn it off.
        if (speedReached) {
            transfer.on();
        } else {
            transfer.off();
        }
        if (enableTransfer.getAsBoolean() && (difference < TransferConstants.RPM_TOLERANCE)) {
            speedReached = true;
        } else if (!enableTransfer.getAsBoolean()) {
            speedReached = false;
        }
    }
}
