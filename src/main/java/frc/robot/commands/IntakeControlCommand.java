package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeControlCommand extends CommandBase {
    private final IntakeSubsystem intake;
    private final DoubleSupplier intakePercent;

    public IntakeControlCommand(IntakeSubsystem intake, DoubleSupplier intakePercent) {
        this.intake = intake;
        this.intakePercent = intakePercent;
        addRequirements(intake);
    }

    public void execute() {
        // TODO: Implement the functionality of the command.
        intake.setPercent(intakePercent.getAsDouble());
    }
}
