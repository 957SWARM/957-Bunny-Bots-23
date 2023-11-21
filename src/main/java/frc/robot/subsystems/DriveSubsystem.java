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
import com.team957.lib.util.GearRatioHelper;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
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

        private final boolean invertDrive;

        private MaxSwerveModule(
                int driveCanId,
                int steerCanId,
                Gearing driveRatio,
                double offsetRadians,
                int driveCurrentLimitAmps,
                int steerCurrentLimitAmps,
                boolean invertDrive,
                String name,
                String subdirName) {
            drive = new CANSparkMax(driveCanId, MotorType.kBrushless);
            drive.restoreFactoryDefaults();
            drive.setSmartCurrentLimit(driveCurrentLimitAmps);

            steer = new CANSparkMax(steerCanId, MotorType.kBrushless);
            drive.restoreFactoryDefaults();
            steer.setSmartCurrentLimit(steerCurrentLimitAmps);

            this.driveRatio = driveRatio;
            this.offsetRadians = offsetRadians;

            setBrakeMode(Constants.DriveConstants.DEFAULT_BRAKE_MODE_ENABLED);

            this.invertDrive = invertDrive;

            brakeModeEnabledLogger =
                    new Logger<>(
                            () -> brakeModeEnabled,
                            HighLevelLogger.getInstance().getLog(),
                            name + "/brakeModeEnabled",
                            subdirName,
                            true,
                            true);

            steerCurrentLogger =
                    new Logger<>(
                            steer::getOutputCurrent,
                            HighLevelLogger.getInstance().getLog(),
                            name + "/steerCurrent_amps",
                            subdirName,
                            true,
                            true);

            driveCurrentLogger =
                    new Logger<>(
                            drive::getOutputCurrent,
                            HighLevelLogger.getInstance().getLog(),
                            name + "/driveCurrent_amps",
                            subdirName,
                            true,
                            true);

            steerAppliedVoltageLogger =
                    new Logger<>(
                            () -> steer.getAppliedOutput() * steer.getBusVoltage(),
                            HighLevelLogger.getInstance().getLog(),
                            name + "/steerAppliedVoltage_volts",
                            subdirName,
                            true,
                            true);

            driveAppliedVoltageLogger =
                    new Logger<>(
                            () -> drive.getAppliedOutput() * drive.getBusVoltage(),
                            HighLevelLogger.getInstance().getLog(),
                            name + "/driveAppliedVoltage_volts",
                            subdirName,
                            true,
                            true);

            steerBusVoltageLogger =
                    new Logger<>(
                            steer::getBusVoltage,
                            HighLevelLogger.getInstance().getLog(),
                            name + "/steerBusVoltage_volts",
                            subdirName,
                            true,
                            true);
            driveBusVoltageLogger =
                    new Logger<>(
                            drive::getBusVoltage,
                            HighLevelLogger.getInstance().getLog(),
                            name + "/driveBusVoltage_volts",
                            subdirName,
                            true,
                            true);

            steerTemperatureLogger =
                    new Logger<>(
                            steer::getMotorTemperature,
                            HighLevelLogger.getInstance().getLog(),
                            name + "/steerTemperature_C",
                            subdirName,
                            true,
                            true);
            driveTemperatureLogger =
                    new Logger<>(
                            drive::getMotorTemperature,
                            HighLevelLogger.getInstance().getLog(),
                            name + "/driveTemperature_C",
                            subdirName,
                            true,
                            true);

            unoffsetSteerLogger =
                    new Logger<>(
                            this::getUnoffsetSteerPositionRadians,
                            HighLevelLogger.getInstance().getLog(),
                            name + "/unoffsetSteer_radians",
                            subdirName,
                            true,
                            true);
            steerLogger =
                    new Logger<>(
                            this::getSteerPositionRadians,
                            HighLevelLogger.getInstance().getLog(),
                            name + "/steer_radians",
                            subdirName,
                            true,
                            true);

            drivePositionLogger =
                    new Logger<>(
                            this::getDriveAccumulationMeters,
                            HighLevelLogger.getInstance().getLog(),
                            name + "/drivePosition_meters",
                            subdirName,
                            true,
                            true);
            driveVelocityLogger =
                    new Logger<>(
                            this::getDriveVelocityMetersPerSecond,
                            HighLevelLogger.getInstance().getLog(),
                            name + "/driveVelocity_meters_per_second",
                            subdirName,
                            true,
                            true);
        }

        private MaxSwerveModule(
                int driveCanId,
                int steerCanId,
                double offsetRadians,
                boolean invertDrive,
                String name) {
            this(
                    driveCanId,
                    steerCanId,
                    Gearing.L3,
                    offsetRadians,
                    Constants.DriveConstants.DRIVE_CURRENT_LIMIT_AMPS,
                    Constants.DriveConstants.STEER_CURRENT_LIMIT_AMPS,
                    invertDrive,
                    name,
                    "swerve");
        }

        public void setDriveControlInput(double volts) {
            drive.setVoltage(
                    UtilityMath.clamp(Constants.MiscConstants.saturationVoltage, volts)
                            * (invertDrive ? -1 : 1));
        }

        public void setSteerControlInput(double volts) {
            steer.setVoltage(-UtilityMath.clamp(Constants.MiscConstants.saturationVoltage, volts));
            // drive motor inverted
        }

        public void setBrakeMode(boolean enabled) {
            drive.setIdleMode(enabled ? IdleMode.kBrake : IdleMode.kCoast);
            steer.setIdleMode(enabled ? IdleMode.kBrake : IdleMode.kCoast);

            brakeModeEnabled = enabled;
        }

        public double getUnoffsetSteerPositionRadians() {
            return steer.getAbsoluteEncoder(Type.kDutyCycle).getPosition();
        }

        public double getSteerPositionRadians() {
            return UtilityMath.normalizeAngleRadians(
                    getUnoffsetSteerPositionRadians() - offsetRadians);
        }

        public double getDriveAccumulationMeters() {
            return driveRatio.ratio.inputFromOutput(drive.getEncoder().getPosition())
                    * 2
                    * Math.PI
                    * Constants.DriveConstants.WHEEL_RADIUS_METERS
                    * (invertDrive ? -1 : 1);
            // apply gear ratio, convert rotations into radians, convert radians into linear
            // distance
        }

        public double getDriveVelocityMetersPerSecond() {
            return driveRatio.ratio.inputFromOutput(
                            Units.rotationsPerMinuteToRadiansPerSecond(
                                    drive.getEncoder().getVelocity()))
                    * Constants.DriveConstants.WHEEL_RADIUS_METERS
                    * (invertDrive ? -1 : 1);
        }

        public SwerveModulePosition getState() {
            return new SwerveModulePosition(
                    getDriveAccumulationMeters(), new Rotation2d(getSteerPositionRadians()));
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

    public record ModuleStates(
            SwerveModulePosition frontLeft,
            SwerveModulePosition frontRight,
            SwerveModulePosition backRight,
            SwerveModulePosition backLeft) {
        /**
         * Returns the module states in an array format. Order compatible with the Kinematics object
         * in Constants.
         *
         * @return The module states as array.
         */
        public SwerveModulePosition[] asArray() {
            return new SwerveModulePosition[] {frontLeft, frontRight, backRight, backLeft};
        }
    }
    ;

    public final MaxSwerveModule frontLeft =
            new MaxSwerveModule(
                    Constants.DriveConstants.FRONT_LEFT_DRIVE_CANID,
                    Constants.DriveConstants.FRONT_LEFT_STEER_CANID,
                    Constants.DriveConstants.FRONT_LEFT_OFFSET_RADIANS,
                    false,
                    "frontLeft");

    public final MaxSwerveModule frontRight =
            new MaxSwerveModule(
                    Constants.DriveConstants.FRONT_RIGHT_DRIVE_CANID,
                    Constants.DriveConstants.FRONT_RIGHT_STEER_CANID,
                    Constants.DriveConstants.FRONT_RIGHT_OFFSET_RADIANS,
                    false,
                    "frontRight");

    public final MaxSwerveModule backRight =
            new MaxSwerveModule(
                    Constants.DriveConstants.BACK_RIGHT_DRIVE_CANID,
                    Constants.DriveConstants.BACK_RIGHT_STEER_CANID,
                    Constants.DriveConstants.BACK_RIGHT_OFFSET_RADIANS,
                    true,
                    "backRight");

    public final MaxSwerveModule backLeft =
            new MaxSwerveModule(
                    Constants.DriveConstants.BACK_LEFT_DRIVE_CANID,
                    Constants.DriveConstants.BACK_LEFT_STEER_CANID,
                    Constants.DriveConstants.BACK_LEFT_OFFSET_RADIANS,
                    true,
                    "backLeft");

    public DriveSubsystem() {
        register();
    }

    @Override
    public void periodic() {
        frontLeft.updateLogs();
        frontRight.updateLogs();
        backLeft.updateLogs();
        backRight.updateLogs();
    }

    public ModuleStates getStates() {
        return new ModuleStates(
                frontLeft.getState(),
                frontRight.getState(),
                backRight.getState(),
                backLeft.getState());
    }
}
