package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class FlywheelControlCommand extends CommandBase{
    private final ShooterSubsystem shooter;
    private final DoubleSupplier targetRPM;

    public FlywheelControlCommand(ShooterSubsystem shooter, DoubleSupplier targetRPM){
        this.shooter = shooter;
        this.targetRPM = targetRPM;
    }

    public void execute(){
        // TODO: Implement the functionality of the command.
    }
}