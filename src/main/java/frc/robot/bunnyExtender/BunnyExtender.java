package frc.robot.bunnyExtender;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface BunnyExtender extends Subsystem {

    // extends piston out
    public void extend();

    // retracts piston in
    public void retract();
}
