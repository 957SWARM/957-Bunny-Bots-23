// It's Blinkin' Time!

package frc.robot.microsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class Blinkin {

    Spark motor;

    public Blinkin(int Channel) {

        motor = new Spark(Channel);
    }

    // No balls?
    public void green() {

        motor.set(0.73);
    }

    // Full balls
    public void red() {

        motor.set(0.61);
    }

    // 1-2 Balls
    public void gold() {

        motor.set(0.67);
    }

    // 3-4 Balls
    public void redOrange() {

        motor.set(0.63);
    }

    public void automation() {

        motor.set(-0.57);
    }

    public void secondAutomation() {

        motor.set(-0.89);
    }
}
