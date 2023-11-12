package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TransferConstants;

public class TransferSubsystem extends SubsystemBase {

    private final TalonSRX motor;
    private final boolean isEnabled;

    public TransferSubsystem() {
        motor = new TalonSRX(TransferConstants.CAN_ID);

        // Default behavior
        isEnabled = false;
    }

    public void on() {
        motor.set(ControlMode.PercentOutput, TransferConstants.TRANSFER_PERCENT_POWER);
    }

    public void off() {
        motor.set(ControlMode.PercentOutput, 0);
    }
}
