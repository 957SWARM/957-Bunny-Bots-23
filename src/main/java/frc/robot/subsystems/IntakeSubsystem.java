package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

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

    private final TalonSRX motor;
    private final DigitalInput input =
            new DigitalInput(Constants.IntakeConstants.BREAKBEAM_DIO_PORT);

    public IntakeSubsystem() {
        motor = new TalonSRX(IntakeConstants.CAN_ID);
        motor.configFactoryDefault();
        // motor.configVoltageCompSaturation(IntakeConstants.VOLTAGE_LIMIT);
        motor.configPeakCurrentLimit(IntakeConstants.CURRENT_LIMIT);
    }

    public boolean isBeamBroken() {
        return (Constants.IntakeConstants.BREAKBEAM_TRUE_WHEN_OCCLUDED)
                ? input.get()
                : !input.get();
    }

    public void setVoltage(double voltage) {
        motor.set(ControlMode.PercentOutput, voltage / motor.getBusVoltage());
    }
}
