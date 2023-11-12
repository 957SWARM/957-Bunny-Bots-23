package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.team957.lib.math.filters.ExponentialMovingAverage;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

/*
 * See the following wiki pages:
 * https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-flywheel.html
 */

/*
 * The shooter subsystem will obey a speed fed to it by the
 * SetFlywheelCommand command. The speed will be constantly
 * updated using a DoubleSupplier, which will change the
 * setpoint of the BangBangController. The periodic()
 * function of the subsystem will set the power to the motor
 * using standard PercentOutput.
 */

public class ShooterSubsystem extends SubsystemBase {

    private final CANSparkMax motor;
    private final Encoder encoder;
    private ExponentialMovingAverage average;

    public ShooterSubsystem() {
        motor = new CANSparkMax(ShooterConstants.CAN_ID, MotorType.kBrushless);

        encoder =
                new Encoder(
                        ShooterConstants.ENC_A, ShooterConstants.ENC_B, ShooterConstants.ENC_REV);

        motor.restoreFactoryDefaults();
        motor.setIdleMode(IdleMode.kCoast);
    }

    public double getRPM() {
        return (encoder.getRate() / 2048.0) / 4.0;
    }

    public boolean aboveThreshold(double average, double threshold) {
        return average > threshold;
    }

    public void periodic() {
        // TODO: define periodic behavior of the subsystem.
        average.calculate(motor.getOutputCurrent());
    }

    public void simulationPeriodic() {
        // TODO: define periodic behavior of the subsystem in a simulation.
    }

    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }
}
