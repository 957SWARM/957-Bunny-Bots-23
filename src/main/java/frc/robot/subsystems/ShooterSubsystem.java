package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.peripherals.UI;

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
    private LinearFilter filter;
    private double filterOutput;

    public ShooterSubsystem() {
        motor = new CANSparkMax(ShooterConstants.CAN_ID, MotorType.kBrushless);

        encoder =
                new Encoder(
                        ShooterConstants.ENC_A, ShooterConstants.ENC_B, ShooterConstants.ENC_REV);

        motor.restoreFactoryDefaults();
        motor.setIdleMode(IdleMode.kCoast);
        motor.setSmartCurrentLimit(ShooterConstants.CURRENT_LIMIT);
        // 2048 encoder ticks per revolution
        encoder.setDistancePerPulse(1.0 / 2048);
        filter = LinearFilter.highPass(.1, .02);
        motor.setInverted(true);
        motor.setIdleMode(IdleMode.kCoast);
    }

    public double getRPM() {
        // converts rps to rpm
        return (encoder.getRate()) * 60;
    }

    public boolean aboveThreshold(double threshold) {
        return filterOutput > threshold;
    }

    public void periodic() {
        filterOutput = filter.calculate(motor.getOutputCurrent());
        SmartDashboard.putNumber("current", filterOutput);
        UI.getInstance().setShooterSpeed(encoder.getRate() * 60);
    }

    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }
}
