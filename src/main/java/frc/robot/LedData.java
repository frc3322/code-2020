package frc.robot;

import edu.wpi.first.wpilibj.I2C;

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

    public static LedData getInstance() {
        return instance;
    }
}