package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TransferConstants;

public class TransferSubsystem extends SubsystemBase {

    private final CANSparkMax motor;
    private final boolean isEnabled;

    public TransferSubsystem() {
        motor = new CANSparkMax(TransferConstants.CAN_ID, MotorType.kBrushless);

        // Default behavior
        isEnabled = false;
    }

    public void periodic() {
        // TODO: define periodic behavior of the subsystem.
    }

    public void simulationPeriodic() {
        // TODO: define periodic behavior of the subsystem in a simulation.
    }
}
