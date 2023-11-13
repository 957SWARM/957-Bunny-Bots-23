// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.team957.lib.math.UtilityMath;
import com.team957.lib.telemetry.HighLevelLogger;
import com.team957.lib.telemetry.Logger;
import com.team957.lib.telemetry.Logger.LoggerFactory;
import com.team957.lib.util.GearRatioHelper;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;

public class DriveSubsystem implements Subsystem {
    public class MaxSwerveModule {
        private final Logger<Boolean> brakeModeEnabledLogger;

        private final Logger<Double> steerCurrentLogger;
        private Logger<Double> driveCurrentLogger;

        private final Logger<Double> steerAppliedVoltageLogger;
        private final Logger<Double> driveAppliedVoltageLogger;

        private final Logger<Double> steerBusVoltageLogger;
        private final Logger<Double> driveBusVoltageLogger;

        private final Logger<Double> steerTemperatureLogger;
        private final Logger<Double> driveTemperatureLogger;

        private final Logger<Double> unoffsetSteerLogger;
        private final Logger<Double> steerLogger;

        private final Logger<Double> drivePositionLogger;
        private final Logger<Double> driveVelocityLogger;

        private boolean brakeModeEnabled = false;

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
                int steerCurrentLimitAmps,
                String name,
                String subdirName) {
            drive = new CANSparkMax(driveCanId, MotorType.kBrushless);
            drive.setSmartCurrentLimit(driveCurrentLimitAmps);

            steer = new CANSparkMax(steerCanId, MotorType.kBrushless);
            steer.setSmartCurrentLimit(steerCurrentLimitAmps);

            this.driveRatio = driveRatio;
            this.offsetRadians = offsetRadians;

            setBrakeMode(Constants.DriveConstants.DEFAULT_BRAKE_MODE_ENABLED);

            brakeModeEnabledLogger =
                    new Logger<>(
                            () -> brakeModeEnabled,
                            HighLevelLogger.getInstance().getLog(),
                            name,
                            subdirName,
                            true,
                            true);

            LoggerFactory<Double> doubleFactory =
                    new LoggerFactory<>(
                            HighLevelLogger.getInstance().getLog(), subdirName, true, true);

            steerCurrentLogger =
                    doubleFactory.getLogger("steerCurrent_amps", steer::getOutputCurrent);
            driveCurrentLogger =
                    doubleFactory.getLogger("driveCurrent_amps", drive::getOutputCurrent);

            steerAppliedVoltageLogger =
                    doubleFactory.getLogger(
                            "steerAppliedVoltage_volts",
                            () -> steer.getAppliedOutput() * steer.getBusVoltage());
            driveAppliedVoltageLogger =
                    doubleFactory.getLogger(
                            "driveAppliedVoltage_volts",
                            () -> drive.getAppliedOutput() * drive.getBusVoltage());

            steerBusVoltageLogger =
                    doubleFactory.getLogger("steerBusVoltage_volts", steer::getBusVoltage);
            driveBusVoltageLogger =
                    doubleFactory.getLogger("driveBusVoltage_volts", drive::getBusVoltage);

            steerTemperatureLogger =
                    doubleFactory.getLogger("steerTemperature_C", steer::getMotorTemperature);
            driveTemperatureLogger =
                    doubleFactory.getLogger("driveTemperature_C", drive::getMotorTemperature);

            unoffsetSteerLogger =
                    doubleFactory.getLogger(
                            "unoffsetSteerAngle_radians", this::getUnoffsetSteerPositionRadians);
            steerLogger =
                    doubleFactory.getLogger("steerAngle_radians", this::getSteerPositionRadians);

            drivePositionLogger =
                    doubleFactory.getLogger(
                            "drivePosition_meters", this::getDriveAccumulationMeters);
            driveVelocityLogger =
                    doubleFactory.getLogger(
                            "driveVelocity_meters_per_second",
                            this::getDriveVelocityMetersPerSecond);
        }

        private MaxSwerveModule(int driveCanId, int steerCanId, double offsetRadians, String name) {
            this(
                    driveCanId,
                    steerCanId,
                    Gearing.L3,
                    offsetRadians,
                    Constants.DriveConstants.DRIVE_CURRENT_LIMIT_AMPS,
                    Constants.DriveConstants.STEER_CURRENT_LIMIT_AMPS,
                    name,
                    "swerve");
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

            brakeModeEnabled = enabled;
        }

        public double getUnoffsetSteerPositionRadians() {
            return UtilityMath.normalizeAngleRadians(
                    steer.getAbsoluteEncoder(Type.kDutyCycle).getPosition() * 2 * Math.PI);
        }

        public double getSteerPositionRadians() {
            return UtilityMath.normalizeAngleRadians(
                    getUnoffsetSteerPositionRadians() - offsetRadians);
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

        private void updateLogs() {
            brakeModeEnabledLogger.update();

            steerCurrentLogger.update();
            driveCurrentLogger.update();

            steerAppliedVoltageLogger.update();
            driveAppliedVoltageLogger.update();

            steerBusVoltageLogger.update();
            driveBusVoltageLogger.update();

            steerTemperatureLogger.update();
            driveTemperatureLogger.update();

            unoffsetSteerLogger.update();
            steerLogger.update();

            drivePositionLogger.update();
            driveVelocityLogger.update();
        }
    }

    public final MaxSwerveModule frontLeft =
            new MaxSwerveModule(
                    Constants.DriveConstants.FRONT_LEFT_DRIVE_CANID,
                    Constants.DriveConstants.FRONT_LEFT_STEER_CANID,
                    Constants.DriveConstants.FRONT_LEFT_OFFSET_RADIANS,
                    "frontLeft");

    public final MaxSwerveModule frontRight =
            new MaxSwerveModule(
                    Constants.DriveConstants.FRONT_RIGHT_DRIVE_CANID,
                    Constants.DriveConstants.FRONT_RIGHT_STEER_CANID,
                    Constants.DriveConstants.FRONT_RIGHT_OFFSET_RADIANS,
                    "frontRight");

    public final MaxSwerveModule backRight =
            new MaxSwerveModule(
                    Constants.DriveConstants.BACK_RIGHT_DRIVE_CANID,
                    Constants.DriveConstants.BACK_RIGHT_STEER_CANID,
                    Constants.DriveConstants.BACK_RIGHT_OFFSET_RADIANS,
                    "backRight");

    public final MaxSwerveModule backLeft =
            new MaxSwerveModule(
                    Constants.DriveConstants.BACK_LEFT_DRIVE_CANID,
                    Constants.DriveConstants.BACK_LEFT_STEER_CANID,
                    Constants.DriveConstants.BACK_LEFT_OFFSET_RADIANS,
                    "backLeft");

    @Override
    public void periodic() {
        frontLeft.updateLogs();
        frontRight.updateLogs();
        backLeft.updateLogs();
        backRight.updateLogs();
    }
}
