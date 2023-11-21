// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.team957.lib.util.DeltaTimeUtil;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.BallPathConstants;
import frc.robot.Constants.BlinkinConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.BasicVisionTargetingCommand;
// import frc.robot.commands.BasicVisionTargetingCommand;
import frc.robot.commands.IntakeControlCommand;
import frc.robot.commands.ShooterControlCommand;
import frc.robot.commands.TransferControlCommand;
import frc.robot.input.DefaultDriver;
import frc.robot.input.DefaultOperator;
import frc.robot.input.DriverInput;
import frc.robot.input.OperatorInput;
import frc.robot.microsystems.IntakeStates;
import frc.robot.microsystems.RobotState;
import frc.robot.microsystems.UI;
import frc.robot.subsystems.BlinkinSubsystem;
import frc.robot.subsystems.BunnyGrabberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransferSubsystem;

public class RobotContainer {
    private DriverInput driver = UI.getInstance().getDriverBinding();
    private OperatorInput operator = UI.getInstance().getOperatorBinding();

    // Subsystems
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();
    private final ShooterSubsystem m_shooter = new ShooterSubsystem();
    private final BunnyGrabberSubsystem m_grabber = new BunnyGrabberSubsystem();
    private final TransferSubsystem m_transfer = new TransferSubsystem();
    private final IntakeSubsystem m_intake = new IntakeSubsystem();
    private final BlinkinSubsystem m_blinkin = new BlinkinSubsystem(BlinkinConstants.BLINKIN_CHANNEL);

    // State Machine
    public RobotState ballPathState = RobotState.IDLE;
    public double currentShooterRPM = 0;
    public double targetShooterRPM = 0;
    public IntakeStates intakeState = IntakeStates.IDLE;
    public boolean enableTransfer = false;

    // Ball Intake
    int ballCount = 0;
    DeltaTimeUtil dtUtilBreakBeam;
    double breakBeamDelay = 0;

    // Shooter
    DeltaTimeUtil dtUtilShooterCurrent;
    DeltaTimeUtil dtUtilShooterWait;    //  allows final ball to be shot before robot becomes idle
    double shooterCurrentDelay = 0;
    double shooterWaitDelay = 0;
    boolean ballLeft = false;

    // Triggers
    // Control Triggers
    Trigger grabberTrigger;
    Trigger shootTrigger;
    Trigger cancelTrigger;
    Trigger intakeTrigger;
    Trigger ejectTrigger;
    Trigger visionTrigger;
    Trigger increaseBallTrigger;
    Trigger decreaseBallTrigger;
    // Sensing Triggers
    Trigger beamBrokenTrigger;
    Trigger currentThresholdTrigger;
    Trigger tooManyBallsTrigger;
    // Blinkin light Triggers
    Trigger greenTrigger;
    Trigger redTrigger;
    Trigger goldTrigger;
    Trigger redOrangeTrigger;
    Trigger automationTrigger;
    Trigger secondAutomationTrigger;


    /*
     * RobotContainer Constructor.
     * Sets up default commands and calls configureBindings()
     */
    public RobotContainer() {
        // timer object
        dtUtilBreakBeam = new DeltaTimeUtil();
        dtUtilShooterCurrent = new DeltaTimeUtil();

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
        // targetShooterRPM));

        m_transfer.setDefaultCommand(
                new TransferControlCommand(
                        m_transfer,
                        () -> enableTransfer,
                        () -> currentShooterRPM,
                        () -> targetShooterRPM));
        // Configure the trigger bindings
        configureBindings();

        m_intake.setDefaultCommand(
            new IntakeControlCommand(
                m_intake, 
                () -> intakeState.voltage())
        );

        m_shooter.setDefaultCommand(
            new ShooterControlCommand(
                m_shooter, 
                () -> targetShooterRPM)
        );
    }

    /*
     * Local function that configures drive controller bindings.
     * See wiki pages on Triggers for documentation.
     */
    private void configureBindings() {

        // CONTROL TRIGGERS
        grabberTrigger = new Trigger(() -> driver.toggleGrabber());
        grabberTrigger.onTrue(m_grabber.toggleBunnyGrabber());

        cancelTrigger = new Trigger(() -> driver.cancel());
        cancelTrigger.onTrue(Commands.runOnce(() -> ballPathState = RobotState.IDLE));

        shootTrigger = new Trigger(() -> driver.shoot());
        shootTrigger.onTrue(Commands.runOnce(() -> ballPathState = RobotState.SHOOT));

        intakeTrigger = new Trigger(() -> driver.intake());
        intakeTrigger.onTrue(Commands.runOnce(() -> ballPathState = RobotState.INTAKE));

        ejectTrigger = new Trigger(() -> driver.intake());
        ejectTrigger.onTrue(Commands.runOnce(() -> ballPathState = RobotState.EJECT));

        visionTrigger = new Trigger(() -> driver.visionTargeting());
        visionTrigger.toggleOnTrue(new BasicVisionTargetingCommand(
                m_robotDrive, 
                () -> -MathUtil.applyDeadband(driver.swerveY(), OIConstants.kDriveDeadband), 
                () -> -MathUtil.applyDeadband(driver.swerveX(), OIConstants.kDriveDeadband)
            )
        );

        increaseBallTrigger = new Trigger(() -> driver.increaseBallCount());
        increaseBallTrigger.onTrue(Commands.runOnce(() -> ballCount++));

        decreaseBallTrigger = new Trigger(() -> driver.decreaseBallCount());
        decreaseBallTrigger.onTrue(Commands.runOnce(() -> ballCount--));
        
        // SENSING TRIGGERS
        // increases ball count if breakbeam sensor detects something. Debounced to prevent rapid changes
        beamBrokenTrigger = new Trigger(() -> m_intake.isBeamBroken());
        beamBrokenTrigger.debounce(BallPathConstants.DEBOUNCE_SENSOR_TIME, Debouncer.DebounceType.kBoth).onTrue(
            Commands.runOnce(() -> ballCount++)
        );

        // decreases ball count if shooter current spikes. Debounced to prevent rapid changes
        currentThresholdTrigger = new Trigger(() -> m_shooter.aboveThreshold(ShooterConstants.DETECTION_THRESHOLD));
        currentThresholdTrigger.debounce(BallPathConstants.DEBOUNCE_CURRENT_TIME, Debouncer.DebounceType.kBoth).onTrue(
            Commands.runOnce(() -> ballCount--)
        );

        // ejects balls if we have more than the max allowed (5)
        tooManyBallsTrigger = new Trigger(() -> ballCount > BallPathConstants.MAX_BALL_COUNT);
        tooManyBallsTrigger.onTrue(Commands.runOnce(() -> ballPathState = RobotState.EJECT));


        // BLINKIN LIGHT TRIGGERS
        greenTrigger = new Trigger(() -> ballCount == BlinkinConstants.GREEN_VALUE);
        greenTrigger.onTrue(m_blinkin.green());
        goldTrigger = new Trigger(() -> ballCount >= BlinkinConstants.GOLD_RANGE_MIN && ballCount <= BlinkinConstants.GOLD_RANGE_MAX);
        goldTrigger.onTrue(m_blinkin.gold());
        redOrangeTrigger = new Trigger(() -> ballCount >= BlinkinConstants.REDORANGE_RANGE_MIN && ballCount <= BlinkinConstants.REDORANGE_RANGE_MAX);
        redOrangeTrigger.onTrue(m_blinkin.redOrange());
        redTrigger = new Trigger(() -> ballCount >= BlinkinConstants.RED_RANGE_UPPERBOUND || ballCount < BlinkinConstants.RED_RANGE_LOWERBOUND);
        redTrigger.onTrue(m_blinkin.red());
    }

    public void stateMachinePeriodic() {

        // switch cases out of switch statement
        switch (ballPathState) {
            case IDLE:

                // transfer off, shooter off, intake off
                enableTransfer = false;
                intakeState = IntakeStates.IDLE;
                targetShooterRPM = 0;
                break;
            case EJECT:

                // transfer off, shooter off, intake eject
                enableTransfer = false;
                intakeState = IntakeStates.EJECT;
                targetShooterRPM = 0;
                break;
            case INTAKE:

                // transfer off, shooter off, intake on
                enableTransfer = false;
                intakeState = IntakeStates.INTAKE;
                targetShooterRPM = 0;
                break;
            case SHOOT:

                // transfer on, shooter on, intake off
                enableTransfer = true;
                intakeState = IntakeStates.IDLE;
                targetShooterRPM = ShooterConstants.SPEED_RPM;
                break;
            default:
                break;
        }


        if(ballPathState == RobotState.SHOOT && ballCount == 0){
            // Need to figure out how to add a delay of .5seconds so that robot can shoot final ball before turning idle
            shooterWaitDelay += dtUtilShooterWait.getTimeSecondsSinceLastCall();
            if (shooterWaitDelay > ShooterConstants.WAIT_DURATION) {
                ballPathState = RobotState.IDLE;
                shooterWaitDelay = 0;
            }
        }

        // pushes ball count to dashboard
        UI.getInstance().setBallCount(ballCount);
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

    public void updateControllers(){
         // Controllers
        driver = UI.getInstance().getDriverBinding();
        operator = UI.getInstance().getOperatorBinding();
    }
}
