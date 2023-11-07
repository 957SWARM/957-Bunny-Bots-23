package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.utils.Constants.IntakeConstants;

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

public class IntakeSubsystem extends SubsystemBase {

    // TODO: add subsystem dependencies
    CANSparkMax motor;

    public IntakeSubsystem() {
        // TODO: implement the constructor.
        motor = new CANSparkMax(IntakeConstants.CAN_ID, MotorType.kBrushed);
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
