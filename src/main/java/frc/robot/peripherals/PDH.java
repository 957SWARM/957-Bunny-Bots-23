package frc.robot.peripherals;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import frc.robot.Constants;

public class PDH {
    public static final PDH instance = new PDH();

    private final PowerDistribution pdh =
            new PowerDistribution(Constants.PDHConstants.PDH_CAN_ID, ModuleType.kRev);

    private PDH() {}

    public double getCurrentAmps(int channel) {
        return pdh.getCurrent(channel);
    }

    public double getTemperatureC() {
        return pdh.getTemperature();
    }

    public double getTotalCurrentAmps() {
        return pdh.getTotalCurrent();
    }

    public double getTotalEnergyJoules() {
        return pdh.getTotalEnergy();
    }

    public double getVoltage() {
        return pdh.getVoltage();
    }

    public void setSwitchableChannel(boolean on) {
        pdh.setSwitchableChannel(on);
    }
}
