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
    boolean bunnyGrabber;
    SendableChooser<Command> autoChooser;
    SendableChooser<DriverInput> driverChooser;
    SendableChooser<OperatorInput> operatorChooser;
    
    private static UI INSTANCE;

    private UI(){
        autoChooser = new SendableChooser<>();
        autoChooser.setDefaultOption("No Auto?", new InstantCommand());

        driverChooser = new SendableChooser<>();
        driverChooser.setDefaultOption("Default Driver", new DefaultDriver(ballCount));

        operatorChooser = new SendableChooser<>();
        operatorChooser.setDefaultOption("Default Operator", new DefaultOperator(ballCount));
    }

    public static UI getInstance(){
        if(INSTANCE == null){
            INSTANCE = new UI();
        }
        return INSTANCE;
    }


    public void periodic(){
        SmartDashboard.putNumber("# of Balls", ballCount);
        SmartDashboard.putNumber("Shooter Speed", shooterSpeed);
        SmartDashboard.putNumber("Speed X", speedX);
        SmartDashboard.putNumber("Speed Y", speedY);
        SmartDashboard.putNumber("Speed Magnitude", speedMagnitude);
        SmartDashboard.putBoolean("Bunny Grabber State", bunnyGrabber);
        SmartDashboard.putData("Auto Selector", autoChooser);
    }

    public void setBallCount(int ballCount){
        this.ballCount = ballCount;
    }

    public void setShooterSpeedNumber(int shooterSpeed){
        this.shooterSpeed = shooterSpeed;
    }

    public void setRobotSpeedX(int speedX){
        this.speedX = speedX;
    }

    public void setRobotSpeedY(int speedY){
        this.speedY = speedY;
    }
    
    public void setRobotSpeedMagnitude(int speedMagnitude){
        this.speedMagnitude = speedMagnitude;
    }

    public void grabberState(boolean bunnyGrabber){
        this.bunnyGrabber = bunnyGrabber;
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
    
    public void setAutonomousCommand(String name, Command autoCommand) {
        autoChooser.addOption(name, autoCommand);
    }
}