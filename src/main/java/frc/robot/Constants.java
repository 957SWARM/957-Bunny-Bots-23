package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class Constants {
    public static final class DriveConstants {
        // Driving Parameters - Note that these are not the maximum capable speeds of
        // the robot, rather the allowed maximum speeds
        public static final double kMaxSpeedMetersPerSecond = 4.8;
        public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

        public static final double kDirectionSlewRate = 1.2; // radians per second
        public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
        public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

        // Chassis configuration
        public static final double kTrackWidth = Units.inchesToMeters(26.5);
        // Distance between centers of right and left wheels on robot
        public static final double kWheelBase = Units.inchesToMeters(26.5);
        // Distance between front and back wheels on robot
        public static final SwerveDriveKinematics kDriveKinematics =
                new SwerveDriveKinematics(
                        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        // Angular offsets of the modules relative to the chassis in radians
        public static final double kFrontLeftChassisAngularOffset = 0; // (-pi/2)
        public static final double kFrontRightChassisAngularOffset = 0;
        public static final double kBackLeftChassisAngularOffset = 0; // (pi)
        public static final double kBackRightChassisAngularOffset = 0; // (pi/2)

        // SPARK MAX CAN IDs
        public static final int kFrontLeftDrivingCanId = 3;
        public static final int kRearLeftDrivingCanId = 5;
        public static final int kFrontRightDrivingCanId = 1;
        public static final int kRearRightDrivingCanId = 7;

        public static final int kFrontLeftTurningCanId = 4;
        public static final int kRearLeftTurningCanId = 6;
        public static final int kFrontRightTurningCanId = 2;
        public static final int kRearRightTurningCanId = 8;

        public static final boolean kGyroReversed = false;
    }

    public static final class ModuleConstants {
        // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
        // This changes the drive speed of the module (a pinion gear with more teeth will result in
        // a
        // robot that drives faster).
        public static final int kDrivingMotorPinionTeeth = 14;

        // Invert the turning encoder, since the output shaft rotates in the opposite direction of
        // the steering motor in the MAXSwerve Module.
        public static final boolean kTurningEncoderInverted = true;

        // Calculations required for driving motor conversion factors and feed forward
        public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
        public static final double kWheelDiameterMeters = 0.0762;
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
        // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on
        // the bevel pinion
        public static final double kDrivingMotorReduction =
                (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
        public static final double kDriveWheelFreeSpeedRps =
                (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction;

        public static final double kDrivingEncoderPositionFactor =
                (kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction; // meters
        public static final double kDrivingEncoderVelocityFactor =
                ((kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction)
                        / 60.0; // meters per second

        public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
        public static final double kTurningEncoderVelocityFactor =
                (2 * Math.PI) / 60.0; // radians per second

        public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
        public static final double kTurningEncoderPositionPIDMaxInput =
                kTurningEncoderPositionFactor; // radians

        public static final double kDrivingP = 0.04;
        public static final double kDrivingI = 0;
        public static final double kDrivingD = 0;
        public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
        public static final double kDrivingMinOutput = -1;
        public static final double kDrivingMaxOutput = 1;

        public static final double kTurningP = 1;
        public static final double kTurningI = 0;
        public static final double kTurningD = 0;
        public static final double kTurningFF = 0;
        public static final double kTurningMinOutput = -1;
        public static final double kTurningMaxOutput = 1;

        public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
        public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

        public static final int kDrivingMotorCurrentLimit = 50; // amps
        public static final int kTurningMotorCurrentLimit = 20; // amps
    }

    public static final class OIConstants {
        public static final double kDriveDeadband = 0.05;
        public static final int kDefaultDriverPort = 0;
        public static final int kDefaultOperatorPort = 1;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

        // Constraint for the motion profiled robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class NeoMotorConstants {
        public static final double kFreeSpeedRpm = 5676;
    }

    public static final class BallPathConstants {
        public static final int MAX_BALL_COUNT = 5;
        public static final double DEBOUNCE_CURRENT_TIME = .3;
        public static final double DEBOUNCE_SENSOR_TIME = .1;
    }

    public static final class ShooterConstants {
        public static final double BB_TOL = 0;
        public static final int ENC_A = 0;
        public static final int ENC_B = 0;
        public static final boolean ENC_REV = false;
        public static final int CAN_ID = 0;
        public static final double RUNNING_VOLTAGE = 11;
        public static final int SPEED_RPM = 3000;
        public static final double WAIT_DURATION = 0.5;
        public static final int DETECTION_THRESHOLD = 55;
        public static final int CURRENT_LIMIT = 10;
    }

    public static final class IntakeConstants {
        public static final double CURRENT_FILTER_RESPONSE_CONSTANT = 0.5;
        public static final double HIGH_CURRENT_THRESHOLD_AMPS = 10;

        public static final int CURRENT_LIMIT = 40;
        public static final int VOLTAGE_LIMIT = 11;

        public static final double RUNNING_VOLTAGE = 9;
        public static final int CAN_ID = 0;

        public static final int BREAKBEAM_DIO_PORT = 0;
        public static final boolean BREAKBEAM_TRUE_WHEN_OCCLUDED = true;
    }

    public static final class TransferConstants {
        public static final int CAN_ID = 0;
        public static final double kp = 0;
        public static final double ki = 0;
        public static final double kd = 0;
        public static final int RPM_TOLERANCE = 20;
        public static final double TRANSFER_PERCENT_POWER = .6;
        public static final int CURRENT_LIMIT = 40;
        public static final int VOLTAGE_LIMIT = 11;
    }

    public static final class VisionTargetingConstants {
        public static final double TARGETING_KP = 0.0;
        public static final double TARGETING_KI = 0.0;
        public static final double TARGETING_KD = 0.0;
    }
    
    public static final class BunnyGrabberConstants {
        public static final int MODULE_PORT = 14;
        public static final int FORWARD_PORT = 0;
        public static final int REVERSE_PORT = 1;
    }

    public static final class BlinkinConstants {
        // Channel needs to be set
        public static final int BLINKIN_CHANNEL = 0;
        public static final double GREEN = 0.73;
        public static final double RED = 0.61;
        public static final double GOLD = 0.67;
        public static final double REDORANGE = 0.63;
        public static final double AUTOMATION = -0.57;
        public static final double SECOND_AUTOMATION = -0.89;

        public static final int GREEN_VALUE = 0;
        public static final int GOLD_RANGE_MIN = 1;
        public static final int GOLD_RANGE_MAX = 2;
        public static final int REDORANGE_RANGE_MIN = 3;
        public static final int REDORANGE_RANGE_MAX = 4;
        public static final int RED_RANGE_UPPERBOUND = 5;
        public static final int RED_RANGE_LOWERBOUND = 0;
    }
}
