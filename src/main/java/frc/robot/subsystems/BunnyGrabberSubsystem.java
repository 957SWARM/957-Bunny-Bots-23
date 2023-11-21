package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
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

    public CommandBase extendBunnyGrabber() {
        return this.runOnce(() -> piston.set(Value.kForward));
    }

    public CommandBase retractBunnyGrabber() {
        return this.runOnce(() -> piston.set(Value.kReverse));
    }

    public CommandBase toggleBunnyGrabber() {
        return this.runOnce(() -> piston.toggle());
    }

    public void periodic() {
        UI.getInstance().grabberState(!(piston.get() == Value.kOff));
    }
}
