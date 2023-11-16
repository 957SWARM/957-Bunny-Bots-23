// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.input.DefaultDriver;
import frc.robot.input.DefaultOperator;
import frc.robot.input.DriverInput;
import frc.robot.input.OperatorInput;
import frc.robot.microsystems.RobotState;
import frc.robot.subsystems.BunnyGrabberSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransferSubsystem;

public class RobotContainer {

    // Controllers
    DriverInput driver = new DefaultDriver(0);
    OperatorInput operator = new DefaultOperator(1);

    // Subsystems
    private final ShooterSubsystem m_shooter = new ShooterSubsystem();
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

        // m_shooter.setDefaultCommand(new FlywheelControlCommand(m_shooter, () ->
        // targetFlywheelRPM));

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
        return null;
    }
}
