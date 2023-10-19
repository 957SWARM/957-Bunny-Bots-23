package frc.robot.bunnyExtender;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class BunnyExtenderController extends CommandBase {

    public BunnyExtenderController(BunnyExtender bunnyExtender) {
        addRequirements(bunnyExtender);
    }
}
