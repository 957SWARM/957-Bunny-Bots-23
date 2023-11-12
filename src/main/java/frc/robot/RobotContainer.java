// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.TransferControlCommand;
import frc.robot.input.DefaultDriver;
import frc.robot.input.DefaultOperator;
import frc.robot.input.DriverInput;
import frc.robot.input.OperatorInput;
import frc.robot.microsystems.RobotState;
import frc.robot.subsystems.BunnyGrabberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import java.util.List;

public class RobotContainer {

    // Controllers
    DriverInput driver = new DefaultDriver(0);
    OperatorInput operator = new DefaultOperator(1);

    // Subsystems
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();
    // private final ShooterSubsystem m_shooter = new ShooterSubsystem();
    private final BunnyGrabberSubsystem m_grabber = new BunnyGrabberSubsystem();
    private final TransferSubsystem m_transfer = new TransferSubsystem();

    // State Machine
    public RobotState ballPathState = RobotState.IDLE;
    public double currentFlywheelRPM = 0;
    public double targetFlywheelRPM = 0;
    public boolean enableIntake = false;
    public boolean enableTransfer = false;

    // Ball Intake
    DigitalInput input;
    int ballCount = 0;
    double breakBeamDelay = 0.0;

    /*
     * RobotContainer Constructor.
     * Sets up default commands and calls configureBindings()
     */
    public RobotContainer() {

        // Configure default commands
        m_robotDrive.setDefaultCommand(
                new RunCommand(
                        () ->
                                m_robotDrive.drive(
                                        -MathUtil.applyDeadband(
                                                driver.swerveY(), OIConstants.kDriveDeadband),
                                        -MathUtil.applyDeadband(
                                                driver.swerveX(), OIConstants.kDriveDeadband),
                                        -MathUtil.applyDeadband(
                                                driver.swerveRot(), OIConstants.kDriveDeadband),
                                        false,
                                        true),
                        m_robotDrive));

        // m_shooter.setDefaultCommand(new FlywheelControlCommand(m_shooter, () ->
        // targetFlywheelRPM));

        m_transfer.setDefaultCommand(
                new TransferControlCommand(
                        m_transfer,
                        () -> enableTransfer,
                        () -> currentFlywheelRPM,
                        () -> targetFlywheelRPM));

        // Configure the trigger bindings
        configureBindings();
    }

    /*
     * Local function that configures drive controller bindings.
     * See wiki pages on Triggers for documentation.
     */
    private void configureBindings() {
        driver.toggleGrabber().onTrue(m_grabber.toggleBunnyGrabber());
    }

    public void stateMachinePeriodic() {
        if (!input.get() && breakBeamDelay > 0.2) {
            breakBeamDelay = 0.0;
            ballCount++;
        }

        switch (ballPathState) {
            case IDLE:
                // TODO: finish state machine
                // TODO: add logic to move between states
                break;
            case EJECT:
                // TODO: finish state machine
                // TODO: add logic to move between states
                break;
            case INTAKE:
                // TODO: finish state machine
                // TODO: add logic to move between states
                break;
            case SHOOT:
                // TODO: finish state machine
                // TODO: add logic to move between states
                break;
            default:
                break;
        }

        breakBeamDelay = +0.02;
    }

    public Command getAutonomousCommand() {

        // Create config for trajectory
        TrajectoryConfig config =
                new TrajectoryConfig(
                                AutoConstants.kMaxSpeedMetersPerSecond,
                                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(DriveConstants.kDriveKinematics);

        // An example trajectory to follow. All units in meters.
        Trajectory exampleTrajectory =
                TrajectoryGenerator.generateTrajectory(
                        // Start at the origin facing the +X direction
                        new Pose2d(0, 0, new Rotation2d(0)),
                        // Pass through these two interior waypoints, making an 's' curve path
                        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                        // End 3 meters straight ahead of where we started, facing forward
                        new Pose2d(3, 0, new Rotation2d(0)),
                        config);

        var thetaController =
                new ProfiledPIDController(
                        AutoConstants.kPThetaController,
                        0,
                        0,
                        AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand =
                new SwerveControllerCommand(
                        exampleTrajectory,
                        m_robotDrive::getPose, // Functional interface to feed supplier
                        DriveConstants.kDriveKinematics,

                        // Position controllers
                        new PIDController(AutoConstants.kPXController, 0, 0),
                        new PIDController(AutoConstants.kPYController, 0, 0),
                        thetaController,
                        m_robotDrive::setModuleStates,
                        m_robotDrive);

        // Reset odometry to the starting pose of the trajectory.
        m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

        // Run path following command, then stop at the end.
        return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
    }
}
