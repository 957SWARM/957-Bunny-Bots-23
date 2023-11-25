// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.team957.lib.util.DeltaTimeUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.BallPathConstants;
import frc.robot.Constants.BlinkinConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.IntakeControlCommand;
import frc.robot.commands.ShooterControlCommand;
import frc.robot.commands.TransferControlCommand;
import frc.robot.input.DefaultDriver;
import frc.robot.input.DefaultOperator;
import frc.robot.input.DriverInput;
import frc.robot.input.OperatorInput;
import frc.robot.peripherals.IMU;
import frc.robot.peripherals.UI;
import frc.robot.subsystems.BlinkinSubsystem;
import frc.robot.subsystems.BunnyGrabberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeStates;

public class RobotContainer {
    public static enum RobotState {
        IDLE,
        EJECT,
        INTAKE,
        SHOOT;
    }

    // Controllers
    DriverInput driver = new DefaultDriver(0);
    OperatorInput operator = new DefaultOperator(1);

    // Subsystems
    private final ShooterSubsystem shooter = new ShooterSubsystem();
    private final BunnyGrabberSubsystem grabber = new BunnyGrabberSubsystem();
    private final TransferSubsystem transfer = new TransferSubsystem();
    private final IntakeSubsystem intake = new IntakeSubsystem();
    private final BlinkinSubsystem blinkin = new BlinkinSubsystem(BlinkinConstants.BLINKIN_CHANNEL);

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
    DeltaTimeUtil dtUtilShooterWait; //  allows final ball to be shot before robot becomes idle
    double shooterCurrentDelay = 0;
    double shooterWaitDelay = 0;
    boolean ballLeft = false;

    // Triggers
    // Control Triggers

    Trigger zeroIMUTrigger;

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

        // m_shooter.setDefaultCommand(new FlywheelControlCommand(m_shooter, () ->
        // targetShooterRPM));

        transfer.setDefaultCommand(
                new TransferControlCommand(
                        transfer,
                        () -> enableTransfer,
                        () -> currentShooterRPM,
                        () -> targetShooterRPM));

        intake.setDefaultCommand(new IntakeControlCommand(intake, () -> intakeState.voltage()));

        shooter.setDefaultCommand(new ShooterControlCommand(shooter, () -> targetShooterRPM));

        configureBindings();
    }

    /*
     * Local function that configures drive controller bindings.
     * See wiki pages on Triggers for documentation.
     */
    private void configureBindings() {

        // TODO: look into removing the instance-level declaration of these
        // idk how the garbage collector treats triggers if their symbol goes out of scope

        // CONTROL TRIGGERS
        zeroIMUTrigger =
                new Trigger(() -> driver.zeroGyro())
                        .onTrue(Commands.runOnce(IMU.instance::setAngleToZero));

        grabberTrigger =
                new Trigger(() -> driver.toggleGrabber()).onTrue(grabber.toggleBunnyGrabber());

        cancelTrigger =
                new Trigger(() -> driver.cancel())
                        .onTrue(Commands.runOnce(() -> ballPathState = RobotState.IDLE));

        shootTrigger =
                new Trigger(() -> driver.shoot())
                        .onTrue(Commands.runOnce(() -> ballPathState = RobotState.SHOOT));

        intakeTrigger =
                new Trigger(() -> driver.intake())
                        .onTrue(Commands.runOnce(() -> ballPathState = RobotState.INTAKE));

        ejectTrigger =
                new Trigger(() -> driver.intake())
                        .onTrue(Commands.runOnce(() -> ballPathState = RobotState.EJECT));

        // visionTrigger =
        //         new Trigger(() -> driver.visionTargeting())
        //                 .toggleOnTrue(
        //                         new BasicVisionTargetingCommand(
        //                                 drive, Limelight.getInstance()::getTx));

        increaseBallTrigger =
                new Trigger(() -> driver.increaseBallCount())
                        .onTrue(Commands.runOnce(() -> ballCount++));

        decreaseBallTrigger =
                new Trigger(() -> driver.decreaseBallCount())
                        .onTrue(Commands.runOnce(() -> ballCount--));

        // SENSING TRIGGERS
        // increases ball count if breakbeam sensor detects something. Debounced to prevent rapid
        // changes
        beamBrokenTrigger =
                new Trigger(() -> intake.isBeamBroken())
                        .debounce(
                                BallPathConstants.DEBOUNCE_SENSOR_TIME,
                                Debouncer.DebounceType.kBoth)
                        .onTrue(Commands.runOnce(() -> ballCount++));

        // decreases ball count if shooter current spikes. Debounced to prevent rapid changes
        currentThresholdTrigger =
                new Trigger(() -> shooter.aboveThreshold(ShooterConstants.DETECTION_THRESHOLD))
                        .debounce(
                                BallPathConstants.DEBOUNCE_CURRENT_TIME,
                                Debouncer.DebounceType.kBoth)
                        .onTrue(Commands.runOnce(() -> ballCount--));

        // ejects balls if we have more than the max allowed (5)
        tooManyBallsTrigger =
                new Trigger(() -> ballCount > BallPathConstants.MAX_BALL_COUNT)
                        .onTrue(Commands.runOnce(() -> ballPathState = RobotState.EJECT));

        // BLINKIN LIGHT TRIGGERS
        greenTrigger =
                new Trigger(() -> ballCount == BlinkinConstants.GREEN_VALUE)
                        .onTrue(blinkin.green());

        // spotless:off
        goldTrigger = new Trigger(() ->
                ballCount >= BlinkinConstants.GOLD_RANGE_MIN && ballCount <= BlinkinConstants.GOLD_RANGE_MAX)
                        .onTrue(blinkin.gold());

        redOrangeTrigger = new Trigger(() ->
                ballCount >= BlinkinConstants.REDORANGE_RANGE_MIN && ballCount <= BlinkinConstants.REDORANGE_RANGE_MAX)
                        .onTrue(blinkin.redOrange());

        redTrigger = new Trigger(() ->
                ballCount >= BlinkinConstants.RED_RANGE_UPPERBOUND || ballCount < BlinkinConstants.RED_RANGE_LOWERBOUND)
                        .onTrue(blinkin.red());
        // spotless:on
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

        if (ballPathState == RobotState.SHOOT && ballCount == 0) {
            // Need to figure out how to add a delay of .5seconds so that robot can shoot final ball
            // before turning idle
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
        return null;
    }

    public void updateControllers() {
        // Controllers
        driver = UI.getInstance().getDriverBinding();
        operator = UI.getInstance().getOperatorBinding();
    }
}
