package frc.robot;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.DriverStation;

public class LedData {
    private I2C ledController;

    private static LedData instance = new LedData(52);

    public LedData(int ArduinoAddress) {
        this.ledController = new I2C(I2C.Port.kOnboard, ArduinoAddress);
    }

    public void startPattern(int patternId) {
        byte[] data = {(byte) patternId};
        this.ledController.writeBulk(data);
    }

    public void setAlliance() {
        DriverStation ds;
        Alliance alliance = ds.getAlliance;
        int allianceId;
        if (alliance == DriverStation::kRed) {
            allianceId = 255;
        } else if (alliance == DriverStation::kBlue) {
            allianceId = 254;
        } else {
            allianceId = 253;
        }
        byte[] data = {(byte) allianceId};
        this.ledController.writeBulk(data);
    }

    public static LedData getInstance() {
        return instance;
    }
}