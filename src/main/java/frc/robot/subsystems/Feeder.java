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
    private int timer = 0;
    private int timeLimit = 30;

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
    }

    public void updateDash() {
        
    }

    @Override
    public void periodic() {
        updateDash();
        feedTop(SmartDashboard.getNumber("Feed Speed Top", 0));
        feedBottom(SmartDashboard.getNumber("Feed Speed Bottom", 0));

        SmartDashboard.putBoolean("AutoFeedTest/Intaking", RobotContainer.intaking);
        SmartDashboard.putBoolean("AutoFeedTest/Shooting", RobotContainer.shooting);
        SmartDashboard.putBoolean("AutoFeedTest/CellSensor", cellSensor.get());
        SmartDashboard.putBoolean("AutoFeedTest/IntookSinceFed", intookSinceFed);
        SmartDashboard.putBoolean("AutoFeedTest/ShotSinceFed", shotSinceFed);
        SmartDashboard.putBoolean("AutoFeedTest/CellSensorGot", cellSensorGot);
        SmartDashboard.putBoolean("AutoFeedTest/FirstTime", firstTime);
        SmartDashboard.putNumber("AutoFeedTest/Timer", timer);
        SmartDashboard.putNumber("AutoFeedTest/TimeLimit", timeLimit);

        if (RobotContainer.intaking && !intookSinceFed) {
            intookSinceFed = true;
        }

        if(RobotContainer.shooting && !shotSinceFed) {
            shotSinceFed = true;
        }

        if (cellSensor.get() && !cellSensorGot){
            cellSensorGot = true;
        }

        if (intookSinceFed) {
            if(firstTime || shotSinceFed){
                if (cellSensorGot) {
                    feedTop(0.8);
                    feedBottom(1);
                    timer++;
                    if(timer > timeLimit) {
                        stop();
                        firstTime = false;
                        intookSinceFed = false;
                        shotSinceFed = false;
                        cellSensorGot = false;
                    }
                }
            }
        }
    }
}