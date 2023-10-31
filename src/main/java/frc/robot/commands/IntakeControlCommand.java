package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeControlCommand extends CommandBase{
    private final IntakeSubsystem intake;
    private final IntSupplier intakeVoltage;

    public IntakeControlCommand(IntakeSubsystem intake, IntSupplier intakeVoltage){
        this.intake = intake;
        this.intakeVoltage = intakeVoltage;
    }

    public void execute(){
        // TODO: Implement the functionality of the command.
        intake.setVoltage(intakeVoltage.getAsInt());
    }
}