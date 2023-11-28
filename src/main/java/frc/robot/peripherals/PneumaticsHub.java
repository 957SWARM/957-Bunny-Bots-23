package frc.robot.peripherals;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Constants;

public class PneumaticsHub {
    public static final PneumaticsHub instance = new PneumaticsHub();

    private final PneumaticHub ph = new PneumaticHub(Constants.PneumaticHubConstants.PH_CAN_ID);

    public double getCompressorCurrentAmps() {
        return ph.getCompressorCurrent();
    }

    public double getAnalogPressurePSI() {
        return ph.getPressure(Constants.PneumaticHubConstants.PRESSURE_SENSOR_ANALOG_CHANNEL);
    }

    public double getTotalSolenoidCurrentAmps() {
        return ph.getSolenoidsTotalCurrent();
    }

    public boolean getDigitalPressureSwitch() {
        return ph.getPressureSwitch();
    }

    public double getInputVoltage() {
        return ph.getInputVoltage();
    }

    public DoubleSolenoid getDoubleSolenoid(int forwardChannel, int backwardsChannel) {
        return ph.makeDoubleSolenoid(forwardChannel, backwardsChannel);
    }

    public Solenoid getSolenoid(int channel) {
        return ph.makeSolenoid(channel);
    }
}
