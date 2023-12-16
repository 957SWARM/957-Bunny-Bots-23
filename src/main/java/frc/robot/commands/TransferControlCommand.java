package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
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
    Timer shootDelay = new Timer();

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
        // System.out.println("Difference: " + difference);
        // if the transfer should be on and the shooter is running at correct speed, turn on
        // transfer. Otherwise, turn it off.
        if (speedReached && shootDelay.get() > .5) {
            transfer.on();
        } else {
            transfer.off();
        }
        // (difference < TransferConstants.RPM_TOLERANCE)
        if (enableTransfer.getAsBoolean()) {
            speedReached = true;
        } else if (!enableTransfer.getAsBoolean()) {
            speedReached = false;
            shootDelay.restart();
        }
    }
}
