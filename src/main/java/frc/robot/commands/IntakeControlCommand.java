package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import java.util.function.DoubleSupplier;

public class IntakeControlCommand extends Command {
    private final IntakeSubsystem intake;
    private final DoubleSupplier intakeVoltage;

    public IntakeControlCommand(IntakeSubsystem intake, DoubleSupplier intakeVoltage) {
        this.intake = intake;
        this.intakeVoltage = intakeVoltage;
        addRequirements(intake);
    }

    public void execute() {
        intake.setVoltage(intakeVoltage.getAsDouble());
    }
}
