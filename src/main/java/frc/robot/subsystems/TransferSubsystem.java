package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.utils.Constants.TransferConstants;

public class TransferSubsystem extends SubsystemBase {

    private final TalonSRX motor;
    private final boolean isEnabled;

    public TransferSubsystem() {
        motor = new TalonSRX(TransferConstants.CAN_ID);

        // Default behavior
        isEnabled = false;
    }

    public void periodic() {
        // TODO: define periodic behavior of the subsystem.
    }

    public void simulationPeriodic() {
        // TODO: define periodic behavior of the subsystem in a simulation.
    }

    // FINISH IMPLEMENTING TRANSFER
    public void set(boolean enable) {
        
    }
}
