package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ShooterHardware implements Shooter {
    CANSparkMax m_shooterMotor = new CANSparkMax(0, MotorType.kBrushless);

    @Override
    public void setFlywheelVoltage(double volts) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setFlywheelVoltage'");
    }

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'periodic'");
    }

    @Override
    public double getFlywheelCurrent() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getFlywheelCurrent'");
    }

    @Override
    public double getFlywheelVelocityRadsPerS() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getFlywheelVelocityRadsPerS'");
    }
}
