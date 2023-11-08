package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
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
    private final BangBangController bb;
    private final Encoder encoder;

    public ShooterSubsystem() {
        motor = new CANSparkMax(ShooterConstants.CAN_ID, MotorType.kBrushless);

        bb = new BangBangController(ShooterConstants.BB_TOL);

        encoder =
                new Encoder(
                        ShooterConstants.ENC_A, ShooterConstants.ENC_B, ShooterConstants.ENC_REV);

        motor.restoreFactoryDefaults();
        motor.setIdleMode(IdleMode.kCoast);
    }

    public double getRPM() {
        // TODO: return the RPM from the encoder.
        return 0;
    }

    public void periodic() {
        // TODO: define periodic behavior of the subsystem.
    }

    public void simulationPeriodic() {
        // TODO: define periodic behavior of the subsystem in a simulation.
    }

    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }
}
