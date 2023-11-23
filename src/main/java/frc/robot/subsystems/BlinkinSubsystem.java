/*It's Blinkin' Time!
Time to Blink all over the place!*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BlinkinConstants;

public class BlinkinSubsystem extends SubsystemBase {

    private final Spark motor;

    public BlinkinSubsystem(int Channel) {

        motor = new Spark(Channel);
    }

    // No balls
    public Command green() {
        return this.runOnce(() -> motor.set(BlinkinConstants.GREEN));
    }

    // Full balls
    public Command red() {

        return this.runOnce(() -> motor.set(BlinkinConstants.RED));
    }

    // 1-2 Balls
    public Command gold() {

        return this.runOnce(() -> motor.set(BlinkinConstants.GOLD));
    }

    // 3-4 Balls
    public Command redOrange() {

        return this.runOnce(() -> motor.set(BlinkinConstants.REDORANGE));
    }

    public Command automation() {

        return this.runOnce(() -> motor.set(BlinkinConstants.AUTOMATION));
    }

    public Command secondAutomation() {

        return this.runOnce(() -> motor.set(BlinkinConstants.SECOND_AUTOMATION));
    }
}
