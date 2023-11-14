package frc.robot.microsystems;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public final class UI {

    int ballCount = 0;
    int shooterSpeed = 0;
    int speedX = 0;
    int speedY = 0;
    int speedMagnitude = 0;
    boolean bunnyGrabber;
    SendableChooser<Command> m_chooser;
    
    private static UI INSTANCE;

    private UI(){
        m_chooser = new SendableChooser<>();
        m_chooser.setDefaultOption("No Auto?", new InstantCommand());
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
        SmartDashboard.putNumber("Robot Speed X", speedX);
        SmartDashboard.putNumber("Robot Speed Y", speedY);
        SmartDashboard.putNumber("Robot Speed Magnitude", speedMagnitude);
        SmartDashboard.putBoolean("State of Bunny Grabber", bunnyGrabber);
        SmartDashboard.putData("Auto Selector", m_chooser);
    }

    public void setBallCount(int ballCount){
        this.ballCount = ballCount;
    }

    public void setShooterSpeedNumber(int shooterSpeed){
        this.shooterSpeed = shooterSpeed;
    }

    public void setRobotSpeed(int speedX, int speedY, int speedMagnitude){
        this.speedX = speedX;
        this.speedY = speedY;
        this.speedMagnitude = speedMagnitude;
    }
    
    public void grabberState(boolean bunnyGrabber){
        bunnyGrabber = false;
        this.bunnyGrabber = bunnyGrabber;
    }

    public Command getAutonomousCommand() {
        return m_chooser.getSelected();
    }
    
    public void setAutonomousCommand(String name, Command autoCommand) {
        m_chooser.addOption(name, autoCommand);
    }
}