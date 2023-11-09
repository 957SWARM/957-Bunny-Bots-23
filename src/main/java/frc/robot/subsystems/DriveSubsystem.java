// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.team957.lib.math.UtilityMath;
import com.team957.lib.util.GearRatioHelper;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;

public class DriveSubsystem implements Subsystem {
    public class MaxSwerveModule {
        private enum Gearing {
            L1(new GearRatioHelper(5.5)),
            L2(new GearRatioHelper(5.08)),
            L3(new GearRatioHelper(4.71));

            final GearRatioHelper ratio;

            Gearing(GearRatioHelper ratio) {
                this.ratio = ratio;
            }
        }

        private final CANSparkMax drive;
        private final CANSparkMax steer;

        private final Gearing driveRatio;
        private final double offsetRadians;

        private MaxSwerveModule(
                int driveCanId,
                int steerCanId,
                Gearing driveRatio,
                double offsetRadians,
                int driveCurrentLimitAmps,
                int steerCurrentLimitAmps) {
            drive = new CANSparkMax(driveCanId, MotorType.kBrushless);
            drive.setSmartCurrentLimit(driveCurrentLimitAmps);

            steer = new CANSparkMax(steerCanId, MotorType.kBrushless);
            steer.setSmartCurrentLimit(steerCurrentLimitAmps);

            this.driveRatio = driveRatio;
            this.offsetRadians = offsetRadians;

            setBrakeMode(Constants.DriveConstants.DEFAULT_BRAKE_MODE_ENABLED);
        }

        private MaxSwerveModule(int driveCanId, int steerCanId, double offsetRadians) {
            this(
                    driveCanId,
                    steerCanId,
                    Gearing.L3,
                    offsetRadians,
                    Constants.DriveConstants.DRIVE_CURRENT_LIMIT_AMPS,
                    Constants.DriveConstants.STEER_CURRENT_LIMIT_AMPS);
            // TODO: may need to invert motors or encoder
        }

        public void setDriveControlInput(double volts) {
            drive.setVoltage(UtilityMath.clamp(Constants.MiscConstants.saturationVoltage, volts));
        }

        public void setSteerControlInput(double volts) {
            steer.setVoltage(UtilityMath.clamp(Constants.MiscConstants.saturationVoltage, volts));
        }

        public void setBrakeMode(boolean enabled) {
            drive.setIdleMode(enabled ? IdleMode.kBrake : IdleMode.kCoast);
            steer.setIdleMode(enabled ? IdleMode.kBrake : IdleMode.kCoast);
        }

        public double getSteerPositionRadians() {
            return UtilityMath.normalizeAngleRadians(
                    (steer.getAbsoluteEncoder(Type.kDutyCycle).getPosition() * 2 * Math.PI)
                            - offsetRadians);
            // convert to radians, apply offset, normalize
        }

        public double getDriveAccumulationMeters() {
            return driveRatio.ratio.outputFromInput(drive.getEncoder().getPosition())
                    * 2
                    * Math.PI
                    * Constants.DriveConstants.WHEEL_RADIUS_METERS;
            // apply gear ratio, convert rotations into radians, convert radians into linear
            // distance
        }

        public double getDriveVelocityMetersPerSecond() {
            return driveRatio.ratio.outputFromInput(drive.getEncoder().getVelocity())
                    * 2
                    * Math.PI
                    * Constants.DriveConstants.WHEEL_RADIUS_METERS;
            // same conversion from above applies even though it's a rate
        }
    }

    public final MaxSwerveModule frontLeft =
            new MaxSwerveModule(
                    Constants.DriveConstants.FRONT_LEFT_DRIVE_CANID,
                    Constants.DriveConstants.FRONT_LEFT_STEER_CANID,
                    Constants.DriveConstants.FRONT_LEFT_OFFSET_RADIANS);

    public final MaxSwerveModule frontRight =
            new MaxSwerveModule(
                    Constants.DriveConstants.FRONT_RIGHT_DRIVE_CANID,
                    Constants.DriveConstants.FRONT_RIGHT_STEER_CANID,
                    Constants.DriveConstants.FRONT_RIGHT_OFFSET_RADIANS);

    public final MaxSwerveModule backRight =
            new MaxSwerveModule(
                    Constants.DriveConstants.BACK_RIGHT_DRIVE_CANID,
                    Constants.DriveConstants.BACK_RIGHT_STEER_CANID,
                    Constants.DriveConstants.BACK_RIGHT_OFFSET_RADIANS);

    public final MaxSwerveModule backLeft =
            new MaxSwerveModule(
                    Constants.DriveConstants.BACK_LEFT_DRIVE_CANID,
                    Constants.DriveConstants.BACK_LEFT_STEER_CANID,
                    Constants.DriveConstants.BACK_LEFT_OFFSET_RADIANS);
}
