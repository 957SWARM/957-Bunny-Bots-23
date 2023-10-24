package frc.robot.bunnyExtender;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Constants;

public class BunnyExtenderHardware implements BunnyExtender {

    DoubleSolenoid piston =
            new DoubleSolenoid(
                    Constants.BunnyExtenderConstants.MODULE_PORT,
                    PneumaticsModuleType.REVPH,
                    Constants.BunnyExtenderConstants.FORWARD_PORT,
                    Constants.BunnyExtenderConstants.REVERSE_PORT);

    @Override
    public void extend() {
        if (piston.get() != Value.kForward) piston.set(Value.kForward);
    }

    @Override
    public void retract() {
        if (piston.get() != Value.kReverse) piston.set(Value.kReverse);
    }
}
