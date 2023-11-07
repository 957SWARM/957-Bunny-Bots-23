package frc.robot.microsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class UI {

    int ballCount = 0;
    int shooterSpeed = 0;
    int speedX = 0;
    int speedY = 0;
    int speedMagnitude = 0;
    boolean bunnyGrabber;
    
    private static UI INSTANCE;

    private UI(){}

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
        SmartDashboard.putString("State of Bunny Grabber", grabberState(bunnyGrabber));
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
    
    public String grabberState(boolean bunnyGrabber){

        if(bunnyGrabber = true){
            return "Extended";
        }
        else{
            return "Retracted";
        }
    }
}
