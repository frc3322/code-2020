package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorSensorV3;
import static frc.robot.Robot.m_can;

public class Feeder extends SubsystemBase {

    CANSparkMax[] motors = new CANSparkMax[2];

    public final int FEED_1 = 0, FEED_2 = 1;

    private final I2C.Port i2cPort = I2C.Port.kOnboard;

    private final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);

    public Feeder() {
        motors[FEED_1] = new CANSparkMax(m_can.FEEDER_1, MotorType.kBrushless);
        motors[FEED_2] = new CANSparkMax(m_can.FEEDER_2, MotorType.kBrushless);

        motors[FEED_2].follow(motors[FEED_1]);
    }

    public void feed(double speed) {
        motors[FEED_1].set(speed);
    }

    public double getIR() {
        return colorSensor.getIR();
    }

    public double getProximity() {
        return colorSensor.getProximity();
    }

    public void putInitialDash() {
        SmartDashboard.putNumber("Feed Speed", 0.2);
    }

    public void updateDash() {
        SmartDashboard.putNumber("IR Value", getIR());
        SmartDashboard.putNumber("Proximity Value", getProximity());
    }

    @Override
    public void periodic() {
        updateDash();
        feed(SmartDashboard.getNumber("Feed Speed", 0));
    }
}