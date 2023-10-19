package frc.robot;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class BunnyExtender extends DoubleSolenoid{
    public BunnyExtender(int forwardChannel, int reverseChannel) {
        super(0, PneumaticsModuleType.REVPH, forwardChannel, reverseChannel);
        // This might need to be a different value
        super.set(kOff);
        //TODO Auto-generated constructor stub
    }

    DoubleSolenoid piston = new DoubleSolenoid(0, PneumaticsModuleType.REVPH, 1, 2);


    // Send to max
    public void extend(){
        if(super.get() != Value.kForward){
            piston.set(kForward);
        }
    }

    // Send to min
    public void retract(){
        if(super.get() != Value.kReverse){
            piston.set(kReverse);
        }
    }


}
