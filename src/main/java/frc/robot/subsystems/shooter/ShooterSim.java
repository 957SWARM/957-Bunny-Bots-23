package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;

public class ShooterSim implements Shooter {
    private final FlywheelSim physics =
            new FlywheelSim(
                    Constants.ShooterConstants.FLYWHEEL_DRIVE,
                    1,
                    Constants.ShooterConstants.FLYWHEEL_MOMENT_KG_M2);

    @Override
    public void setFlywheelVoltage(double volts) {
        physics.setInputVoltage(volts);
    }

    @Override
    public void periodic() {
        physics.update(Constants.Misc.LOOP_PERIOD_S);
    }

    @Override
    public double getFlywheelCurrent() {
        return physics.getCurrentDrawAmps();
    }

    @Override
    public double getFlywheelVelocityRadsPerS() {
        return physics.getAngularVelocityRadPerSec();
    }
}
