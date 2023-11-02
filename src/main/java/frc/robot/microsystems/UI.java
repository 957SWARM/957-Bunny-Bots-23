package frc.robot.microsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class UI {

    int ballCount = 0;
    
    private static UI INSTANCE;

    private UI(){}

    public static UI getInstance(){
        if(INSTANCE == null){
            INSTANCE = new UI();
        }
        return INSTANCE;
    }


    public void periodic(){
        SmartDashboard.putNumber("#", 0);
    }

    public void setBallCount(int ballCount){
        this.ballCount = ballCount;
    }
}
