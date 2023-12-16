// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.team957.lib.util.DeltaTimeUtil;
import edu.wpi.first.util.InterpolatingTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.BlinkinConstants;
import frc.robot.commands.IntakeControlCommand;
import frc.robot.commands.PIDFshooterControlCommand;
import frc.robot.commands.TransferControlCommand;
import frc.robot.input.DefaultDriver;
import frc.robot.input.DefaultOperator;
import frc.robot.input.DriverInput;
import frc.robot.input.OperatorInput;
import frc.robot.peripherals.IMU;
import frc.robot.peripherals.Limelight;
import frc.robot.peripherals.UI;
import frc.robot.subsystems.BlinkinSubsystem;
import frc.robot.subsystems.BlinkinSubsystem.BlinkinState;
import frc.robot.subsystems.BunnyGrabberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeStates;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransferSubsystem;
import java.util.Map.Entry;

public class RobotContainer {
    public static enum RobotState {
        IDLE,
        EJECT,
        INTAKE,
        SHOOT;
    }

    private final InterpolatingTreeMap<Double, Double> distanceToRPMMap =
            new InterpolatingTreeMap<>();

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
    double shooterMultiplier = .93;

    // Triggers
    // Control Triggers

    Trigger zeroIMUTrigger;
    Trigger increaseShootPowerTrigger;
    Trigger decreaseShootPowerTrigger;
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

    // Blinkin
    double signalLightDelay = 0;

    /*
     * RobotContainer Constructor.
     * Sets up default commands and calls configureBindings()
     */
    public RobotContainer() {
        for (Entry<Double, Double> entry :
                Constants.VisionTargetingConstants.DISTANCE_TO_RPM_CONTAINER.entrySet()) {
            distanceToRPMMap.put(entry.getKey(), entry.getValue());
        }

        // timer object
        dtUtilBreakBeam = new DeltaTimeUtil();
        dtUtilShooterCurrent = new DeltaTimeUtil();
        dtUtilShooterWait = new DeltaTimeUtil();

        // m_shooter.setDefaultCommand(new FlywheelControlCommand(m_shooter, () ->
        // targetShooterRPM));

        transfer.setDefaultCommand(
                new TransferControlCommand(
                        transfer,
                        () -> enableTransfer,
                        () -> shooter.getRPM(),
                        () -> targetShooterRPM));

        intake.setDefaultCommand(new IntakeControlCommand(intake, () -> intakeState.voltage()));

        shooter.setDefaultCommand(new PIDFshooterControlCommand(shooter, () -> targetShooterRPM));

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
                        .onTrue(
                                Commands.runOnce(
                                        () -> {
                                            ballPathState = RobotState.IDLE;
                                        }));

        shootTrigger =
                new Trigger(() -> driver.shoot())
                        .onTrue(Commands.runOnce(() -> ballPathState = RobotState.SHOOT));

        intakeTrigger =
                new Trigger(() -> driver.intake())
                        .onTrue(Commands.runOnce(() -> ballPathState = RobotState.INTAKE));

        ejectTrigger =
                new Trigger(() -> driver.eject())
                        .onTrue(Commands.runOnce(() -> ballPathState = RobotState.EJECT));

        increaseShootPowerTrigger =
                new Trigger(() -> operator.increaseShootPower())
                        .onTrue(
                                Commands.runOnce(
                                        () -> {
                                            shooterMultiplier += .01;
                                        }));

        decreaseShootPowerTrigger =
                new Trigger(() -> operator.decreaseShootPower())
                        .onTrue(Commands.runOnce(() -> shooterMultiplier -= .01));

        // visionTrigger =
        //         new Trigger(() -> driver.visionTargeting())
        //                 .toggleOnTrue(
        //                         new BasicVisionTargetingCommand(
        //                                 drive, Limelight.getInstance()::getTx));

        // SENSING TRIGGERS
        // increases ball count if breakbeam sensor detects something. Debounced to prevent rapid
        // changes
        beamBrokenTrigger =
                new Trigger(() -> intake.isBeamBroken())
                        .onTrue(
                                blinkin.blinkCommand(
                                                BlinkinState.GREEN, BlinkinState.OFF, 0.25, 0.25)
                                        .withTimeout(1));

        // decreases ball count if shooter current spikes. Debounced to prevent rapid changes
        /*
        currentThresholdTrigger =
                new Trigger(() -> shooter.aboveThreshold(ShooterConstants.DETECTION_THRESHOLD))
                        .debounce(
                                BallPathConstants.DEBOUNCE_CURRENT_TIME,
                                Debouncer.DebounceType.kBoth)
                        .onTrue(Commands.runOnce(() -> {
                                        ballCount--;
                                        System.out.println("woah!");
                                }));
                        */

        // BLINKIN LIGHT TRIGGERS

        // spotless:on
    }

    public void stateMachinePeriodic() {

        // switch cases out of switch statement
        switch (ballPathState) {
            case IDLE:

                // transfer off, shooter off, intake off
                enableTransfer = false;
                intakeState = IntakeStates.IDLE;
                targetShooterRPM = 3000;
                break;
            case EJECT:

                // transfer off, shooter off, intake eject
                enableTransfer = false;
                intakeState = IntakeStates.EJECT;
                targetShooterRPM = 3000;
                break;
            case INTAKE:

                // transfer off, shooter off, intake on
                enableTransfer = false;
                intakeState = IntakeStates.INTAKE;
                targetShooterRPM = 3000;
                break;
            case SHOOT:

                // transfer on, shooter on, intake off
                enableTransfer = true;
                intakeState = IntakeStates.IDLE;
                targetShooterRPM =
                        shooterMultiplier
                                * distanceToRPMMap.get(
                                        Limelight.getInstance().getDistanceFromTarget());
                break;
            default:
                break;
        }
        SmartDashboard.putNumber("Shooter Multiplier", shooterMultiplier);
        /*
        if (ballPathState == RobotState.SHOOT && ballCount == 0) {
            // Need to figure out how to add a delay of .5seconds so that robot can shoot final ball
            // before turning idle
            shooterWaitDelay += dtUtilShooterWait.getTimeSecondsSinceLastCall();
            if (shooterWaitDelay > ShooterConstants.WAIT_DURATION) {
                ballPathState = RobotState.IDLE;
                shooterWaitDelay = 0;
            }
        }
        */

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
