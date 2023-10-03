package frc.robot;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {
    // Network Table Setup
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("limelight");
    DoubleSubscriber vSub = table.getDoubleTopic("tv").subscribe(0);
    DoubleSubscriber xSub = table.getDoubleTopic("tx").subscribe(0);
    DoubleSubscriber ySub = table.getDoubleTopic("ty").subscribe(0);
    DoubleSubscriber aSub = table.getDoubleTopic("ta").subscribe(0);
    DoubleSubscriber sSub = table.getDoubleTopic("ts").subscribe(0);
    DoubleSubscriber lSub = table.getDoubleTopic("tl").subscribe(0);
    DoubleSubscriber shortSub = table.getDoubleTopic("tshort").subscribe(0);
    DoubleSubscriber longSub = table.getDoubleTopic("tlong").subscribe(0);
    DoubleSubscriber horSub = table.getDoubleTopic("thor").subscribe(0);
    DoubleSubscriber classSub = table.getDoubleTopic("tclass").subscribe(0);
    DoubleSubscriber vertSub = table.getDoubleTopic("tvert").subscribe(0);
    DoubleSubscriber jsonSub = table.getDoubleTopic("tjson").subscribe(0);
    DoubleSubscriber getpipeSub = table.getDoubleTopic("getpipe").subscribe(0);

    // Whether or not a target is seen
    public double getTv(){
        double v = vSub.get();
        return v;
    }

    // Angle off from target horizontally
    public double getTx(){
        double x = xSub.get();
        return x;
    }

    // Angle off from target vertically
    public double getTy(){
        double y = ySub.get();
        return y;
    }

    public double getTa(){
        double a = aSub.get();
        return a;
    }

    public double getTs(){
        double s = sSub.get();
        return s;
    }

    public double getTl(){
        double l = lSub.get();
        return l;
    }

    public double getTshort(){
        double tshort = shortSub.get();
        return tshort;
    }
    
    public double getTlong(){
        double tlong = longSub.get();
        return tlong;
    }
        
    public double getThor(){
        double thor = horSub.get();
        return thor;
    }
    
    public double getTvert(){
        double tvert = vertSub.get();
        return tvert;
    }

    public double getPipe(){
        double getpipe = getpipeSub.get();
        return getpipe;
    }

    public double getjson(){
        double json = jsonSub.get();
        return json;
    }
    
        public double getTclass(){
        double tclass = classSub.get();
        return tclass;
    }

    
    
}

