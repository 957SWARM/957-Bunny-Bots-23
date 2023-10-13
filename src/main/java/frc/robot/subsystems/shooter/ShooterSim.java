package frc.robot.subsystems.shooter;

import com.team957.lib.math.filters.DifferentiatingFilter;
import com.team957.lib.util.DeltaTimeUtil;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;

public class ShooterSim implements Shooter {
    private final FlywheelSim physics =
            new FlywheelSim(
                    Constants.ShooterConstants.FLYWHEEL_DRIVE,
                    1,
                    Constants.ShooterConstants.FLYWHEEL_MOMENT_KG_M2);

    private final DifferentiatingFilter accelFilter =
            new DifferentiatingFilter(); // this is not meant to be very accurate, just an
    // approximate reference for profiling

    private final DeltaTimeUtil dtUtil;

    public ShooterSim() {
        dtUtil = new DeltaTimeUtil();
    }

    @Override
    public void setFlywheelVoltage(double volts) {
        physics.setInputVoltage(volts);
    }

    @Override
    public void periodic() {
        double dt = dtUtil.getTimeSecondsSinceLastCall();

        physics.update(dt);

        accelFilter.calculate(getFlywheelVelocityRadsPerS(), dt);
    }

    @Override
    public double getFlywheelCurrent() {
        return physics.getCurrentDrawAmps();
    }

    @Override
    public double getFlywheelVelocityRadsPerS() {
        return physics.getAngularVelocityRadPerSec();
    }

    @Override
    public double getFlywheelAccelerationRadsPerSSquard() {
        return accelFilter.getCurrentOutput();
    }
}
