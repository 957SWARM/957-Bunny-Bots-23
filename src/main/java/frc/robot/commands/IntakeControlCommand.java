package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.microsystems.IntakeStates;
import frc.robot.subsystems.IntakeSubsystem;
import java.util.function.DoubleSupplier;

public class IntakeControlCommand extends CommandBase {
    private final IntakeSubsystem intake;
    private final DoubleSupplier intakeVoltage;

    public IntakeControlCommand(
            IntakeSubsystem intake, DoubleSupplier intakeVoltage) {
        this.intake = intake;
        this.intakeVoltage = intakeVoltage;
        addRequirements(intake);
    }

    public void execute() {
        intake.setVoltage(intakeVoltage.getAsDouble());
    }
}
