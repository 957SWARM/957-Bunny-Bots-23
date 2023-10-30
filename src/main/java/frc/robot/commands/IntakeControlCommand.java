package frc.robot.commands;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeControlCommand extends CommandBase{
    private final IntakeSubsystem intake;
    private final BooleanSupplier intakeOn;

    public IntakeControlCommand(IntakeSubsystem intake, BooleanSupplier intakeOn){
        this.intake = intake;
        this.intakeOn = intakeOn;
    }

    public void execute(){
        // TODO: Implement the functionality of the command.
    }
}