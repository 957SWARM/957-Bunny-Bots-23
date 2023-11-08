package frc.robot.microsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class IMU {
    private final AHRS navx = new AHRS();

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
        return navx.getRotation2d();
    }

    public double getAngularVelocityRadiansPerSecond() {
        return Units.degreesToRadians(navx.getRate())
                * (Constants.IMUConstants.YAW_AXIS_INVERTED ? -1 : 1);
    }
}
