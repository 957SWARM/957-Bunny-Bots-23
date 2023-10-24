package frc.robot;

public class VisionSubsystem {
    
    double tapeTx;

    public void lockOnTarget(Drivetrain drivetrain, Limelight limelight){
        tapeTx = limelight.getTx();
        //double rotationSpeedMultiplier = 0.1 * (Math.sqrt(Math.abs(limelight.getTx()))) ;
        //double rotationSpeedMultiplier = 0.67 * Math.log(Math.abs(limelight.getTx())-1);
        double rotationSpeedMultiplier = 0.29 * (Math.atan(Math.abs(limelight.getTx()) - 9)) + 0.54;
        if(tapeTx > 0.5){
            drivetrain.drive(0, 0, -1 * rotationSpeedMultiplier, false);
        }
        else if(tapeTx < -0.5){
            drivetrain.drive(0, 0, 1 * rotationSpeedMultiplier, false);
        }
        else{
            drivetrain.drive(0, 0, 0, false);
        }
    }
}
