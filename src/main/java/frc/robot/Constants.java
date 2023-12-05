package frc.robot;

import com.team957.lib.controllers.feedback.PID.PIDConstants;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class Constants {
    public static final class MiscConstants {
        public static final double saturationVoltage = 12;
    }

    public static final class DriveConstants {
        public static final double WHEEL_MAX_SPEED_METERS_PER_SECOND = 3;

        public static final Translation2d FRONT_LEFT_TRANSFORM = new Translation2d(-0.325, 0.325);
        public static final Translation2d FRONT_RIGHT_TRANSFORM = new Translation2d(0.325, 0.325);
        public static final Translation2d BACK_RIGHT_TRANSFORM = new Translation2d(0.325, -0.325);
        public static final Translation2d BACK_LEFT_TRANSFORM = new Translation2d(-0.325, -0.325);

        // ORDER OF THESE ARGUMENTS MATTERS!!!!!
        // all calls to this kinematics object must match it
        public static final SwerveDriveKinematics KINEMATICS =
                new SwerveDriveKinematics(
                        FRONT_LEFT_TRANSFORM,
                        FRONT_RIGHT_TRANSFORM,
                        BACK_RIGHT_TRANSFORM,
                        BACK_LEFT_TRANSFORM);

        public static final PIDConstants STEER_FEEDBACK_CONSTANTS = new PIDConstants(2.5, 0, 0);
        public static final PIDConstants DRIVE_FEEDBACK_CONSTANTS = new PIDConstants(1, 7.5, 0);

        // look into feedforward again
        public static final double DRIVE_FEEDFORWARD_KS_VOLTS = 0;
        public static final double DRIVE_FEEDFORWARD_KV_VOLT_SECONDS_PER_METER = 0;

        public static final int DRIVE_CURRENT_LIMIT_AMPS = 50;
        public static final int STEER_CURRENT_LIMIT_AMPS = 20;

        public static final int FRONT_LEFT_DRIVE_CANID = 3;
        public static final int FRONT_LEFT_STEER_CANID = 4;

        public static final int FRONT_RIGHT_DRIVE_CANID = 1;
        public static final int FRONT_RIGHT_STEER_CANID = 2;

        public static final int BACK_RIGHT_DRIVE_CANID = 7;
        public static final int BACK_RIGHT_STEER_CANID = 8;

        public static final int BACK_LEFT_DRIVE_CANID = 5;
        public static final int BACK_LEFT_STEER_CANID = 6;

        // the offset is the "raw" value reported when the module is at the "zero" position

        // last updated: November 15 8:30 PM
        public static final double FRONT_LEFT_OFFSET_RADIANS = 2.62 - (Math.PI / 2);
        public static final double FRONT_RIGHT_OFFSET_RADIANS = 0;
        public static final double BACK_RIGHT_OFFSET_RADIANS = -(Math.PI / 2);
        public static final double BACK_LEFT_OFFSET_RADIANS = 0;

        public static final boolean DEFAULT_BRAKE_MODE_ENABLED = false;

        public static final double WHEEL_RADIUS_METERS =
                Units.inchesToMeters(1.5); // wheel DIAMETER is 3 in
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
        public static final int ENC_B = 1;
        public static final boolean ENC_REV = false;
        public static final int CAN_ID = 11;
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
        public static final int CAN_ID = 10;

        public static final int BREAKBEAM_DIO_PORT = 2;
        public static final boolean BREAKBEAM_TRUE_WHEN_OCCLUDED = true;
    }

    public static final class TransferConstants {
        public static final int CAN_ID = 9;
        public static final double kp = 0;
        public static final double ki = 0;
        public static final double kd = 0;
        public static final int RPM_TOLERANCE = 20;
        public static final double TRANSFER_PERCENT_POWER = .6;
        public static final int CURRENT_LIMIT = 40;
        public static final int VOLTAGE_LIMIT = 11;
    }

    public static final class VisionTargetingConstants {
        // radians per second per radians
        public static final double TARGETING_KP = 1;
        public static final double TARGETING_KI = 0;
        public static final double TARGETING_KD = 0;

        // this is just kP because:
        // 1: this is for a second-order cascaded controller
        // 2: the controller doesn't need to be that accurate
        // 3: it's much easier to program the command if it's stateless (like a pure P controller)
    }

    public static final class BunnyGrabberConstants {
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

    public static final class IMUConstants {
        public static final boolean YAW_AXIS_INVERTED = true;
        // this should NOT be applied to the navx.getRotation2D(), that's already offset
        // this is for the angular rate
    }

    public static final class PDHConstants {
        public static final int PDH_CAN_ID = 30;
        public static final int PCH_CAN_ID = 31;
    }

    public static final class PneumaticHubConstants {
        public static final int PH_CAN_ID = 31;

        public static final int PRESSURE_SENSOR_ANALOG_CHANNEL = 0;
    }
}
