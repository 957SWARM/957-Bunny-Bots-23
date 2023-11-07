package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TransferSubsystem;
import java.util.function.BooleanSupplier;

public class TransferControlCommand extends CommandBase {
    private final TransferSubsystem transfer;
    private final BooleanSupplier enableTransfer;

    public TransferControlCommand(TransferSubsystem transfer, BooleanSupplier enableTransfer) {
        this.transfer = transfer;
        this.enableTransfer = enableTransfer;
    }

    public void execute() {
        // TODO: Implement the functionality of the command.
    }
}
