package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.microsystems.UI;

public class BunnyGrabberSubsystem extends SubsystemBase {

    private final DoubleSolenoid piston;

    public BunnyGrabberSubsystem() {
        piston =
                new DoubleSolenoid(
                        Constants.BunnyGrabberConstants.MODULE_PORT,
                        PneumaticsModuleType.REVPH,
                        Constants.BunnyGrabberConstants.FORWARD_PORT,
                        Constants.BunnyGrabberConstants.REVERSE_PORT);
        piston.set(Value.kReverse);
    }

    public Command extendBunnyGrabber() {
        return this.runOnce(() -> piston.set(Value.kForward));
    }

    public Command retractBunnyGrabber() {
        return this.runOnce(() -> piston.set(Value.kReverse));
    }

    public Command toggleBunnyGrabber() {
        return this.runOnce(() -> piston.toggle());
    }

    public void periodic() {
        if (piston.get() == Value.kForward) {
            UI.getInstance().grabberState(true);
        } else {
            UI.getInstance().grabberState(false);
        }
    }
}
