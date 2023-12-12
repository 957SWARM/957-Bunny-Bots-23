package frc.robot.peripherals;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class IMU {
    public static final IMU instance = new IMU();

    private IMU() {}

    private final Pigeon2 pigeon = new Pigeon2(0);

    private Rotation2d angleOffset = new Rotation2d();
    // this is the number ADDED to the raw value to get the corrected angle

    public void setAngleOffset(Rotation2d angleOffset) {
        this.angleOffset = angleOffset;
    }

    public Rotation2d getAngleOffset() {
        return angleOffset;
    }

    /** Sets the yaw offset such that the currrent state is "zero". */
    public void setAngleToZero() {
        angleOffset = angleOffset.minus(getCorrectedAngle());
    }

    public Rotation2d getCorrectedAngle() {
        return getRawAngle().plus(angleOffset);
    }

    public Rotation2d getRawAngle() {
        return Rotation2d.fromDegrees(pigeon.getYaw());
    }

    public double getAngularVelocityRadiansPerSecond() {
        return Units.degreesToRadians(0) // TODO
                * (Constants.IMUConstants.YAW_AXIS_INVERTED ? -1 : 1);
    }
}
