/*It's Blinkin' Time!
Time to Blink all over the place!*/

package frc.robot.microsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.utils.Constants.BlinkinConstants;

public class Blinkin {

    Spark motor;

    public Blinkin(int Channel) {

        motor = new Spark(Channel);
    }

    // No balls
    public void green() {

        motor.set(BlinkinConstants.GREEN);
    }

    // Full balls
    public void red() {

        motor.set(BlinkinConstants.RED);
    }

    // 1-2 Balls
    public void gold() {

        motor.set(BlinkinConstants.GOLD);
    }

    // 3-4 Balls
    public void redOrange() {

        motor.set(BlinkinConstants.REDORANGE);
    }

    public void automation() {

        motor.set(BlinkinConstants.AUTOMATION);
    }

    public void secondAutomation() {

        motor.set(BlinkinConstants.SECOND_AUTOMATION);
    }
}
