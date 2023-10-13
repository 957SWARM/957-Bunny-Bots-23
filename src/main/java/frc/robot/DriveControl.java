package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

public class DriveControl extends Joystick {

    public DriveControl(int port) {
        super(port);
        // TODO Auto-generated constructor stub
    }

    final int shoot = 1; // the A button, what do you think it does
    final int extra = 2; // the B button, idk use for whatever
    final int intake = 3; // the X button, intake duh
    final int another = 4; // the Y button, another extra button
    final int xAxisDrive = 0; // 0 axis
    final int yAxisDrive = 1; // 1 axis
    final int gTurnAxis = 4; // 4 axis
    final int rightStickY = 5; // 5 axis

    public boolean getShoot() {
        return super.getRawButton(shoot);
    }

    public boolean getExtra() {
        return super.getRawButton(extra);
    }

    public boolean getIntake() {
        return super.getRawButton(intake);
    }

    public boolean getAnother() {
        return super.getRawButton(another);
    }

    public double getXAxisDrive() {
        return -super.getRawAxis(xAxisDrive);
    }

    public double getYAxisDrive() {
        return -super.getRawAxis(yAxisDrive);
    }

    public double getGTurnAxis() {
        return -super.getRawAxis(gTurnAxis);
    }

    public double getRightStickY() {
        return super.getRawAxis(rightStickY);
    }
}
