package frc.robot;

import edu.wpi.first.math.controller.PIDController;

public class VisionSubsystem {

    double tapeTx;
    PIDController rotationPID = new PIDController(0.01, 0, 0);
    double testPIDOutput = rotationPID.calculate(tapeTx, 0);

    public void lockOnTarget(Drivetrain drivetrain, Limelight limelight) {
        tapeTx = limelight.getTx();
        // double rotationSpeedMultiplier = 0.1 * (Math.sqrt(Math.abs(limelight.getTx()))) ;
        // double rotationSpeedMultiplier = 0.67 * Math.log(Math.abs(limelight.getTx())-1);
        // double rotationSpeedMultiplier = 0.29 * (Math.atan(Math.abs(limelight.getTx()) - 9)) + 0.54;
        
        //Fix PID Stuff (I dont know what im doing)
        if (tapeTx > 0.5) {
            drivetrain.drive(0, 0, rotationPID.calculate(tapeTx, 0), false);
        } else {
            drivetrain.drive(0, 0, 0, false);
        }
    }
}
