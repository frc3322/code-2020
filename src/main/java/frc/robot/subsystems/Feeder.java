package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import static frc.robot.Robot.m_can;

public class Feeder extends SubsystemBase {

    CANSparkMax[] motors = new CANSparkMax[2];

    private final int FEED_1 = 0, FEED_2 = 1;

    DigitalInput cellSensor;

    private boolean firstTime = true;
    private boolean shotSinceFed = false;
    private boolean intookSinceFed = false;
    private boolean checkedIntake = false;
    private boolean checkedShooter = false;
    private boolean checkedCellSensor = false;
    private boolean cellSensorGot = false;
    //private int timer = 0;
    //private int timeLimit = 1;

    public Feeder() {
        motors[FEED_1] = new CANSparkMax(m_can.FEEDER_1, MotorType.kBrushless);
        motors[FEED_2] = new CANSparkMax(m_can.FEEDER_2, MotorType.kBrushless);

        motors[FEED_1].setSmartCurrentLimit(20, 15);
        motors[FEED_2].setSmartCurrentLimit(20, 15);

        cellSensor = new DigitalInput(Constants.RobotMap.DIO.IR_ID);
    }
    
    public boolean getIR() {
            return cellSensor.get();
    }

    public void setGotFalse() {
        cellSensorGot = false;
    }

    public void feedTop(double speed) {
        motors[FEED_1].set(speed);
    }

    public void feedBottom(double speed) {
        motors[FEED_2].set(speed);
    }

    public void stop() {
        feedTop(0.0);
        feedBottom(0.0);
    }

    public void putInitialDash() {
        SmartDashboard.putNumber("Feed Speed Top", 0);
        SmartDashboard.putNumber("Feed Speed Bottom", 0);
        setGotFalse();
    }

    public void updateDash() {
        
    }

    @Override
    public void periodic() {
        updateDash();
        feedTop(SmartDashboard.getNumber("Feeder/Feed Speed Top", 0));
        feedBottom(SmartDashboard.getNumber("Feeder/Feed Speed Bottom", 0));

        SmartDashboard.putBoolean("Feeder/AutoFeedTest/Intaking", RobotContainer.intaking);
        SmartDashboard.putBoolean("Feeder/AutoFeedTest/Shooting", RobotContainer.shooting);
        SmartDashboard.putBoolean("Feeder/AutoFeedTest/CellSensor", !cellSensor.get());
        SmartDashboard.putBoolean("Feeder/AutoFeedTest/IntookSinceFed", intookSinceFed);
        SmartDashboard.putBoolean("Feeder/AutoFeedTest/ShotSinceFed", shotSinceFed);
        SmartDashboard.putBoolean("Feeder/AutoFeedTest/CellSensorGot", cellSensorGot);
        SmartDashboard.putBoolean("Feeder/AutoFeedTest/FirstTime", firstTime);
        // SmartDashboard.putNumber("Feeder/AutoFeedTest/Timer", timer);
        // SmartDashboard.putNumber("Feeder/AutoFeedTest/TimeLimit", timeLimit);

        if (RobotContainer.intaking) {
            intookSinceFed = true;
        }

        if (RobotContainer.shooting) {
            shotSinceFed = true;
        }

        if (!cellSensor.get()) {
            cellSensorGot = true;
        }

        if (intookSinceFed) {
            if (firstTime || shotSinceFed) {
                feedTop(0.5);
                feedBottom(.5);
                if (cellSensorGot) {
                    //timer++;
                    //if (timer > timeLimit) {
                        stop();
                        firstTime = false;
                        intookSinceFed = false;
                        shotSinceFed = false;
                        cellSensorGot = false;
                        //timer = 0;
                    //}
                }
            }
        }
    }
}