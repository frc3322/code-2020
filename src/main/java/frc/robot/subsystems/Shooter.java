/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SpeedControllerGroup;

import static frc.robot.Robot.m_can;

import java.util.HashMap;

public class Shooter extends SubsystemBase {

    private static double P = 0.00051;
    private static double I = 0.000001;
    private static double D = 30;
    private static double F = 0;

    private CANSparkMax[] motors = new CANSparkMax[2];
    private CANEncoder[] encoders = new CANEncoder[2];

    private final int MOTOR_0 = 0, MOTOR_1 = 1;

    private double[] distances = {0, 1, 2, 3, 4};

    private double[] RPMs = {3000, 3000, 3300, 3100};

    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    private NetworkTableEntry tx = table.getEntry("tx");
    private NetworkTableEntry ty = table.getEntry("ty");
    private NetworkTableEntry ta = table.getEntry("ta");

    private double limelightX = tx.getDouble(0.0);
    private double limelightY = ty.getDouble(0.0);
    private double limelightA = ta.getDouble(0.0);

    CANPIDController controller;

    public Shooter() {
        motors[MOTOR_0] = new CANSparkMax(m_can.SHOOTER_1, MotorType.kBrushless);
        motors[MOTOR_1] = new CANSparkMax(m_can.SHOOTER_2, MotorType.kBrushless);

        motors[MOTOR_1].follow(motors[MOTOR_0], true);

        encoders[MOTOR_0] = new CANEncoder(motors[MOTOR_0]);
        encoders[MOTOR_1] = new CANEncoder(motors[MOTOR_1]);

        controller = motors[MOTOR_0].getPIDController();

        controller.setP(P);
        controller.setI(I);
        controller.setD(D);
        controller.setFF(F);

        controller.setOutputRange(0, 1);

        updateConstants();
    }

    public void setSetpoint(double setpoint) {
        controller.setReference(setpoint, ControlType.kVelocity);
    }

    public double getDistance() {
        double limelightAngle = SmartDashboard.getNumber("Limelight Angle", 45);
        double targetAngle = limelightY;
        double limelightHeight = SmartDashboard.getNumber("Limelight Height", 1/2);
        double targetHeight = (7 + (5/6));

        return ((targetHeight-limelightHeight)/(Math.tan((limelightAngle + targetAngle) * Math.PI/180)));
    }

    public double getRPM() {
        double myNumber = getDistance();
        double distance = Math.abs(distances[0] - myNumber);
        int idx = 0;
        for(int c = 1; c < distances.length; c++){
            double cdistance = Math.abs(distances[c] - myNumber);
            if(cdistance < distance){
                idx = c;
                distance = cdistance;
            }
        }

        if(idx <= distances.length && idx <= RPMs.length){
            return RPMs[idx];
        } else {
            return 0;
        }
    }

    public double publishRPM() {
        SmartDashboard.putNumber("Shooter RPM", encoders[MOTOR_0].getVelocity());
        return encoders[MOTOR_0].getVelocity();
    }

    public void setSpeed(double speed) {
        motors[MOTOR_0].set(speed);
    }

    public void stop(){
        motors[MOTOR_0].setVoltage(0);
    }

    public void putInitialDash(){
        SmartDashboard.putNumber("Shooter Speed", 0);
        SmartDashboard.putNumber("Shooter P", P);
        SmartDashboard.putNumber("Shooter I", I);
        SmartDashboard.putNumber("Shooter D", D); 
        SmartDashboard.putNumber("Shooter F", F);
        SmartDashboard.putNumber("Shooter Setpoint", 3000);
        SmartDashboard.putNumber("Robot Distance", 0);
    }

    public void updateConstants() {
        SmartDashboard.putNumber("A PID value", controller.getP());
        controller.setP(SmartDashboard.getNumber("Shooter P", P));
        controller.setI(SmartDashboard.getNumber("Shooter I", I));
        controller.setD(SmartDashboard.getNumber("Shooter D", D));
        controller.setFF(SmartDashboard.getNumber("Shooter F", F));
    }

    @Override
    public void periodic() {
        //setSpeed(SmartDashboard.getNumber("Shooter Speed", 0));
        publishRPM();
    }
}
