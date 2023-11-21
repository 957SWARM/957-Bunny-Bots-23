package frc.robot.microsystems;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.input.DefaultDriver;
import frc.robot.input.DefaultOperator;
import frc.utils.DriverInput;
import frc.utils.OperatorInput;

public final class UI {

    int ballCount = 0;
    int shooterSpeed = 0;
    int speedX = 0;
    int speedY = 0;
    int speedMagnitude = 0;
    boolean bunnyGrabber = false;
    SendableChooser<Command> autoChooser;
    SendableChooser<DriverInput> driverChooser;
    SendableChooser<OperatorInput> operatorChooser;

    private static UI INSTANCE;

    private UI() {
        autoChooser = new SendableChooser<>();
        autoChooser.setDefaultOption("No Auto?", new InstantCommand());

        driverChooser = new SendableChooser<>();
        driverChooser.setDefaultOption("Default Driver", new DefaultDriver(0));

        operatorChooser = new SendableChooser<>();
        operatorChooser.setDefaultOption("Default Operator", new DefaultOperator(1));
    }

    public static UI getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new UI();
        }
        return INSTANCE;
    }

    public void periodic() {
        SmartDashboard.putNumber("# of Balls", ballCount);
        SmartDashboard.putNumber("Shooter Speed", shooterSpeed);
        SmartDashboard.putNumber("Speed X", speedX);
        SmartDashboard.putNumber("Speed Y", speedY);
        SmartDashboard.putNumber("Speed Magnitude", speedMagnitude);
        SmartDashboard.putBoolean("Bunny Grabber State", bunnyGrabber);
        SmartDashboard.putData("Auto Selector", autoChooser);
        SmartDashboard.putData("Driver Selector", driverChooser);
        SmartDashboard.putData("Operator Selector", operatorChooser);
    
    }

    public void setBallCount(int ballCount) {
        this.ballCount = ballCount;
    }

    public void setShooterSpeedNumber(int shooterSpeed) {
        this.shooterSpeed = shooterSpeed;
    }

    public void setRobotSpeedX(int speedX) {
        this.speedX = speedX;
    }

    public void setRobotSpeedY(int speedY) {
        this.speedY = speedY;
    }

    public void setRobotSpeedMagnitude(int speedMagnitude) {
        this.speedMagnitude = speedMagnitude;
    }

    public void grabberState(boolean bunnyGrabber) {
        this.bunnyGrabber = bunnyGrabber;
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public void setAutonomousCommand(String name, Command autoCommand) {
        autoChooser.addOption(name, autoCommand);
    }
    public DriverInput getDriverBinding() {
        return driverChooser.getSelected();
    }

    public void setDriverBinding(String name, DriverInput driverInput) {
        driverChooser.addOption(name, driverInput);
    }
    public OperatorInput getOperatorBinding() {
        return operatorChooser.getSelected();
    }

    public void setOperatorBinding(String name, OperatorInput operatorInput) {
        operatorChooser.addOption(name, operatorInput);
    }
}
