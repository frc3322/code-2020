package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Robot.m_can;

public class Feeder extends SubsystemBase {

    CANSparkMax[] motors = new CANSparkMax[2];

    private final int FEED_1 = 0, FEED_2 = 1;

    public Feeder() {
        motors[FEED_1] = new CANSparkMax(m_can.FEEDER_1, MotorType.kBrushless);
        motors[FEED_2] = new CANSparkMax(m_can.FEEDER_2, MotorType.kBrushless);

        motors[FEED_1].setSmartCurrentLimit(20, 15);
        motors[FEED_2].setSmartCurrentLimit(20, 15);
    }

    public void feedTop(double speed) {
        motors[FEED_1].set(speed);
    }

    public void feedBottom(double speed) {
        motors[FEED_2].set(speed);
    }

    public void putInitialDash() {
        SmartDashboard.putNumber("Feed Speed Top", 0);
        SmartDashboard.putNumber("Feed Speed Bottom", 0);
    }

    public void updateDash() {
        
    }

    @Override
    public void periodic() {
        updateDash();
        feedTop(SmartDashboard.getNumber("Feed Speed Top", 0));
        feedBottom(SmartDashboard.getNumber("Feed Speed Bottom", 0));
    }
}