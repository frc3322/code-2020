/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import static frc.robot.Robot.m_can;

public class Hopper extends SubsystemBase {
    /**
     * Creates a new Hopper.
     */
    private CANSparkMax[] motors = new CANSparkMax[2];
    private CANEncoder[] encoders = new CANEncoder[2];

    DigitalInput cellSensor;
  
    private int LEFT = 0, RIGHT = 1;

    private double timer = 0;
    private double timerSeconds = timer/50;
    private double timeLimit = 5;

    public Hopper() {
        motors[LEFT] = new CANSparkMax(m_can.LEFT_HOPPER_MOTOR, MotorType.kBrushless);
        motors[RIGHT] = new CANSparkMax(m_can.RIGHT_HOPPER_MOTOR, MotorType.kBrushless);

        encoders[LEFT] = motors[LEFT].getEncoder();
        encoders[RIGHT] = motors[RIGHT].getEncoder();

        motors[LEFT].setSmartCurrentLimit(20, 15);
        motors[RIGHT].setSmartCurrentLimit(20, 15);

        cellSensor = new DigitalInput(Constants.RobotMap.DIO.IR_ID);
    }

    public void putInitialDash() {
        SmartDashboard.putNumber("Left Hopper Speed", 0);
        SmartDashboard.putNumber("Right Hopper Speed", 0);
    }

    public void cycle(double m_leftSpeed, double m_rightSpeed) {
        double leftSpeed = 0;
        double rightSpeed = 0;
        timer++;
        if ((timer/50) > 2){
            timer = 0;
        } else if ((timer/50) > 1) {
            leftSpeed = m_leftSpeed;
            rightSpeed = m_rightSpeed;
        } 
        motors[LEFT].set(leftSpeed);
        motors[RIGHT].set(rightSpeed);
    }

    public boolean getIR() {
        return cellSensor.get();
    }

    public void stop() {
        motors[LEFT].set(0);
        motors[RIGHT].set(0);
    }

    public double getVoltage(int n) {
        return motors[n].getBusVoltage();
    }

    public double getMotorHeat(int n) {
        return motors[n].getMotorTemperature();
    }

    public double getOutputCurrent(int n) {
        return motors[n].getOutputCurrent();
    }

    public double getEncoder(int n) {
        return encoders[n].getPosition();
    }

    public double getVelocity(int n) {
        return encoders[n].getVelocity();
    }

    @Override
    public void periodic() {
        //agitate();

        // if(intook) {
        //     if(!cellSensor.get()) {
        //         cycle(-1, -1);
        //     } else if (cellSensor.get()) {
        //         stop();
        //         intook = false;
        //     } else if (cellSensor == null){
        //         cycle(-1, -1);
        //         timer++;
        //         if(timer > timeLimit){
        //             stop();
        //             intook = false;
        //             timer = 0;
        //         }
        //     }
            
        // }
    }
}
