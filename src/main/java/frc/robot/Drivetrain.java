package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class Drivetrain {
    public static final double kMaxSpeed = 3.0;
    public static final double kMaxAngularSpeed = Math.PI;

    private final Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
    private final Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
    private final Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
    private final Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

    private final MAXSwerveModule m_frontLeft;
    private final MAXSwerveModule m_frontRight;
    private final MAXSwerveModule m_backLeft;
    private final MAXSwerveModule m_backRight;

    private final AHRS m_gyro = new AHRS();

    private final SwerveDriveKinematics m_kinematics =
            new SwerveDriveKinematics(
                    m_frontLeftLocation,
                    m_frontRightLocation,
                    m_backLeftLocation,
                    m_backRightLocation);

    private final SwerveDriveOdometry m_odometry;

    public Drivetrain(double fl_offset, double fr_offset, double bl_offset, double br_offset) {

        m_frontLeft = new MAXSwerveModule(3, 4, (fl_offset - .25) * 6.28);
        m_frontRight = new MAXSwerveModule(1, 2, (fr_offset) * 6.28);
        m_backLeft = new MAXSwerveModule(5, 6, (bl_offset + .5) * 6.28);
        m_backRight = new MAXSwerveModule(7, 8, (br_offset + .25) * 6.28);

        m_odometry =
                new SwerveDriveOdometry(
                        m_kinematics,
                        m_gyro.getRotation2d(),
                        new SwerveModulePosition[] {
                            m_frontLeft.getPosition(),
                            m_frontRight.getPosition(),
                            m_backLeft.getPosition(),
                            m_backRight.getPosition()
                        });

        m_gyro.reset();
    }

    /**
     * Returns the pose estimated by wheel odometry and gyroscope measurements.
     *
     * <p>Note that this is subject to a significant amount of drift during matches and should not
     * be trusted (without vision corrections) except for before the bumpers have touched anything.
     *
     * <p>The initial pose of the odometry may also not be consistent with the conventional field
     * coordinate system (as of 10/13).
     *
     * @return A Pose2d (x and y in meters).
     */
    public Pose2d getOdometryPose() {
        return m_odometry.getPoseMeters();
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed Speed of the robot in the x direction (forward).
     * @param ySpeed Speed of the robot in the y direction (sideways).
     * @param rot Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the field.
     */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        var swerveModuleStates =
                m_kinematics.toSwerveModuleStates(
                        fieldRelative
                                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                        xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                                : new ChassisSpeeds(xSpeed, ySpeed, rot));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_backLeft.setDesiredState(swerveModuleStates[2]);
        m_backRight.setDesiredState(swerveModuleStates[3]);
    }

    /** Updates the field relative position of the robot. */
    public void updateOdometry() {
        m_odometry.update(
                m_gyro.getRotation2d(),
                new SwerveModulePosition[] {
                    m_frontLeft.getPosition(),
                    m_frontRight.getPosition(),
                    m_backLeft.getPosition(),
                    m_backRight.getPosition()
                });
    }
}
