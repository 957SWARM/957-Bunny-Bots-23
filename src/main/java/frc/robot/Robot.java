// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.team957.lib.telemetry.BaseHardwareLogger;
import com.team957.lib.telemetry.HighLevelLogger;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.VisionTargetingConstants;
import frc.robot.commands.drivetrain.ChassisControlCommand;
import frc.robot.input.DefaultDriver;
import frc.robot.input.DriverInput;
import frc.robot.peripherals.IMU;
import frc.robot.peripherals.Limelight;
import frc.robot.peripherals.LimelightLib;
import frc.robot.peripherals.UI;
import frc.robot.subsystems.DriveSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private RobotContainer m_robotContainer;
    private UI ui = UI.getInstance();
    double timerControllerUpdate = 0;

    private final DriverInput driver = new DefaultDriver(0);
    private final DriveSubsystem drive = new DriveSubsystem();

    Trigger visionTrigger;

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        m_robotContainer = new RobotContainer();

        HighLevelLogger.getInstance().startLogging();
        HighLevelLogger.getInstance().autoGenerateLogs("highLevel", "base");

        BaseHardwareLogger.getInstance().autoGenerateLogs("baseHardware", "base");

        IMU.instance.setAngleToZero();

        // prevent robot freakout on first vision button press??
        Limelight.getInstance().getTx();
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
        m_robotContainer.stateMachinePeriodic();

        HighLevelLogger.getInstance().updateLogs();
        BaseHardwareLogger.getInstance().updateLogs();

        ui.periodic();
        timerControllerUpdate += .02;
        if (timerControllerUpdate >= 1) {
            m_robotContainer.updateControllers();
            timerControllerUpdate = 0;
        }

        System.out.println(getDistanceFromTarget());
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    /**
     * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        // m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }

        // temporary workaround for commandscheduler requirements issues
        drive.setDefaultCommand(
                new ChassisControlCommand(
                        drive, driver::swerveX, driver::swerveY, () -> getRotationVelocity()));
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {}

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {}

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {}

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {}

    public double getRotationVelocity() {
        // System.out.println(LimelightLib.getTX("limelight"));
        if (driver.visionTargeting()) {
            double kp = VisionTargetingConstants.TARGETING_KP;
            double minCommand = VisionTargetingConstants.TARGETING_MIN_COMMAND;
            double tx = -LimelightLib.getTX("limelight");
            if (Math.abs(tx) > 1) {
                if (tx > 0) {
                    return kp * tx + minCommand;
                } else {
                    return kp * tx - minCommand;
                }
            }
            return 0;
        } else {
            return driver.swerveRot();
        }
    }

    public double getDistanceFromTarget() {
        double angleToGoalDegrees =
                VisionTargetingConstants.LIMELIGHT_ANGLE + LimelightLib.getTY("limelight");
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
        double distanceFromLimelightToGoalMeters =
                (VisionTargetingConstants.TARGET_HEIGHT - VisionTargetingConstants.LIMELIGHT_HEIGHT)
                        / Math.tan(angleToGoalRadians);
        return distanceFromLimelightToGoalMeters;
    }
}
