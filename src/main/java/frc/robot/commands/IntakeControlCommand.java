package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.microsystems.IntakeStates;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeControlCommand extends CommandBase {
    private final IntakeSubsystem intake;
    private final DoubleSupplier intakePercent;
    private final IntakeStates state;

    public IntakeControlCommand(IntakeSubsystem intake, DoubleSupplier intakePercent, IntakeStates state) {
        this.intake = intake;
        this.intakePercent = intakePercent;
        this.state = state;
        addRequirements(intake);
    }

    public void execute() {
        // TODO: Implement the functionality of the command.
        intake.setVoltage(state.voltage());
    }
}
