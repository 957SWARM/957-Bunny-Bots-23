/*It's Blinkin' Time!
Time to Blink all over the place!*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BlinkinSubsystem extends SubsystemBase {

    public static enum BlinkinState {
        GREEN(0.73),
        RED(0.61),
        GOLD(0.67),
        BLUE(0.87),
        RED_ORANGE(0.63),
        AUTOMATION(-0.57),
        SECOND_AUTOMATION(-0.89),
        OFF(0.99);

        private final double dutyCycle;

        BlinkinState(double dutyCycle) {
            this.dutyCycle = dutyCycle;
        }
    }

    private final Spark motor;

    private double speed = 0;

    public BlinkinSubsystem(int Channel) {

        motor = new Spark(Channel);
    }

    @Override
    public void periodic() {
        motor.set(speed);
    }

    public CommandBase setColorCommand(BlinkinState state) {
        return this.runOnce(() -> speed = state.dutyCycle);
    }

    public CommandBase blinkCommand(
            BlinkinState stateA,
            BlinkinState stateB,
            double stateADurationSeconds,
            double stateBDurationSeconds) {
        return new CommandBase() {
            final Timer timer = new Timer();

            @Override
            public void initialize() {
                timer.restart();
            }

            @Override
            public void execute() {
                if (timer.get() % (stateADurationSeconds + stateBDurationSeconds)
                        < stateADurationSeconds) {
                    speed = stateA.dutyCycle;
                } else {
                    speed = stateB.dutyCycle;
                }
            }
            ;
        };
    }
}
