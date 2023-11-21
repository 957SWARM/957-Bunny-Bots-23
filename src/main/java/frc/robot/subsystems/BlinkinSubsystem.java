/*It's Blinkin' Time!
Time to Blink all over the place!*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BlinkinConstants;

public class BlinkinSubsystem extends SubsystemBase {

    private final Spark motor;

    public BlinkinSubsystem(int Channel) {

        motor = new Spark(Channel);
    }

    // No balls
    public CommandBase green() {
        return this.runOnce(() -> motor.set(BlinkinConstants.GREEN));
    }

    // Full balls
    public CommandBase red() {

        return this.runOnce(() -> motor.set(BlinkinConstants.RED));
    }

    // 1-2 Balls
    public CommandBase gold() {

        return this.runOnce(() -> motor.set(BlinkinConstants.GOLD));
    }

    // 3-4 Balls
    public CommandBase redOrange() {

        return this.runOnce(() -> motor.set(BlinkinConstants.REDORANGE));
    }

    public CommandBase automation() {

        return this.runOnce(() -> motor.set(BlinkinConstants.AUTOMATION));
    }

    public CommandBase secondAutomation() {

        return this.runOnce(() -> motor.set(BlinkinConstants.SECOND_AUTOMATION));
    }
}
