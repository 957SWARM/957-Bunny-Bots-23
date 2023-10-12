
import edu.wpi.first.wpilibj.Joystick;

public class DriveController extends Joystick {

    public DriveController(int port) {
        super(port);
        //TODO Auto-generated constructor stub
    }
    
    final int shoot = 1; // the A button, what do you think it does
    final int extra = 2; // the B button, idk use for whatever
    final int intake = 3; // the X button, intake duh
    final int another = 4; // the Y button, another extra button
    final int xAxisDrive = 0; // 0 axis
	final int yAxisDrive = 1; // 1 axis
	final int gTurnAxis = 4; // 4 axis 
    final int rightStickY = 5; // 5 axis

    public boolean getShoot(){
        return super.getRawButton(shoot);
    }
        
    public boolean getExtra(){
        return super.getRawButton(extra);
    }
    
    public boolean getIntake(){
        return super.getRawButton(intake);
    }
    
    public boolean getAnother(){
        return super.getRawButton(another);
    }

    public boolean getXAxisDrive(){
        return super.getRawAxis(xAxisDrive);
    }

    public boolean getYAxisDrive(){
        return -super.getRawAxis(yAxisDrive);
    }

    public boolean getGTurnAxis(){
        return super.getRawAxis(gTurnAxis);

    }
    
    public boolean getRightStickY(){
        return super.getRawAxis(rightStickY);
    }
}