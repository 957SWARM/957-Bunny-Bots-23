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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import monologue.Logged;
import monologue.Monologue.LogBoth;

public class DriveSubsystem implements Subsystem, Logged {
    public class MaxSwerveModule implements Logged {
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

        @LogBoth
        private boolean brakeModeEnabled = false;

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

        @LogBoth
        public double getUnoffsetSteerPositionRadians() {
            return steer.getAbsoluteEncoder(Type.kDutyCycle).getPosition();
        }

        @LogBoth
        public double getSteerPositionRadians() {
            return UtilityMath.normalizeAngleRadians(
                    getUnoffsetSteerPositionRadians() - offsetRadians);
        }

        @LogBoth
        public double getDriveAccumulationMeters() {
            return driveRatio.ratio.inputFromOutput(drive.getEncoder().getPosition())
                    * 2
                    * Math.PI
                    * Constants.DriveConstants.WHEEL_RADIUS_METERS
                    * (invertDrive ? -1 : 1);
            // apply gear ratio, convert rotations into radians, convert radians into linear
            // distance
        }

        @LogBoth
        public double getDriveVelocityMetersPerSecond() {
            return driveRatio.ratio.inputFromOutput(
                            Units.rotationsPerMinuteToRadiansPerSecond(
                                    drive.getEncoder().getVelocity()))
                    * Constants.DriveConstants.WHEEL_RADIUS_METERS
                    * (invertDrive ? -1 : 1);
        }

        @LogBoth
        public SwerveModulePosition getState() {
            return new SwerveModulePosition(
                    getDriveAccumulationMeters(), new Rotation2d(getSteerPositionRadians()));
        }

        @LogBoth
        public double getDriveCurrentAmps() {
                return drive.getOutputCurrent();
        }

        @LogBoth
        public double getSteerCurrentAmps() {
                return steer.getOutputCurrent();
        }

        @LogBoth
        public double getDriveControlEffortVolts() {
                return drive.getAppliedOutput() * drive.getBusVoltage();
        }

        @LogBoth
        public double getSteerControlEffortVolts() {
                return steer.getAppliedOutput() * steer.getBusVoltage();
        }

        @LogBoth
        public double getDriveTemperatureC() {
                return drive.getMotorTemperature();
        }

        @LogBoth
        public double getSteerTemperatureC() {
                return steer.getMotorTemperature();
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

    public ModuleStates getStates() {
        return new ModuleStates(
                frontLeft.getState(),
                frontRight.getState(),
                backRight.getState(),
                backLeft.getState());
    }

    public Command setBrakeModeCommand(boolean enabled) {
        return Commands.runOnce(
                        () -> {
                            for (MaxSwerveModule module :
                                    new MaxSwerveModule[] {
                                        frontLeft, frontRight, backRight, backLeft
                                    }) module.setBrakeMode(enabled);
                        })
                .ignoringDisable(true);
    }
}
