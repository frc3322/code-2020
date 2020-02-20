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
import frc.robot.Constants;

import static frc.robot.Robot.m_can;

public class Shooter extends SubsystemBase {

    private static double P = 0.00051;
    private static double I = 0.000001;
    private static double D = 30;
    private static double F = 0;

    private CANSparkMax[] motors = new CANSparkMax[2];
    private CANEncoder[] encoders = new CANEncoder[2];

    private final int MOTOR_0 = 0, MOTOR_1 = 1;

    private double[] distances = {10};

    private double[] RPMs = {3200};

    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    private NetworkTableEntry ty = table.getEntry("ty");

    private double limelightY = ty.getDouble(0.0);

    CANPIDController controller;

    public Shooter() {
        motors[MOTOR_0] = new CANSparkMax(m_can.SHOOTER_1, MotorType.kBrushless);
        motors[MOTOR_1] = new CANSparkMax(m_can.SHOOTER_2, MotorType.kBrushless);

        motors[MOTOR_0].restoreFactoryDefaults();
        motors[MOTOR_1].restoreFactoryDefaults();

        motors[MOTOR_0].setInverted(false);
        motors[MOTOR_1].setInverted(false);

        motors[MOTOR_0].setIdleMode(IdleMode.kCoast);
        motors[MOTOR_1].setIdleMode(IdleMode.kCoast);

        motors[MOTOR_0].setSmartCurrentLimit(50, 40);
        motors[MOTOR_1].setSmartCurrentLimit(50, 40);

        motors[MOTOR_1].follow(motors[MOTOR_0], true);
        
        motors[MOTOR_0].burnFlash();
        motors[MOTOR_1].burnFlash();

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

    public void initPos() {
        stop();
    }

    public void setSetpoint(double setpoint) {
        controller.setReference(setpoint, ControlType.kVelocity);
    }

    public double getDistance() {
        limelightY = ty.getDouble(0.0);
        double limelightAngle = 10;
        double targetAngle = limelightY;
        SmartDashboard.putNumber("Limelight/Target Angle", targetAngle);
        double limelightHeight = (2+(1/12));
        double targetHeight = (7 + (5/6));

        return ((targetHeight-limelightHeight)/(Math.tan((limelightAngle + targetAngle) * Math.PI/180)));
    }

    public double findRPM() {
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

        if(idx < distances.length && idx < RPMs.length){
            return RPMs[idx];
        } else {
            return 0;
        }
    }

    public double publishRPM() {
        SmartDashboard.putNumber("Shooter/Shooter RPM", encoders[MOTOR_0].getVelocity());
        return encoders[MOTOR_0].getVelocity();
    }

    public void setSpeed(double speed) {
        motors[MOTOR_0].set(speed);
    }

    public void stop(){
        motors[MOTOR_0].setVoltage(0);
    }

    public boolean onTarget(double setpoint) {
        return Math.abs(encoders[MOTOR_0].getVelocity() - setpoint) < Constants.ShooterConstants.SHOOTER_TOLERANCE;
    }

    public void putInitialDash(){
        SmartDashboard.putNumber("Shooter/Shooter Speed", 0);
        SmartDashboard.putNumber("Shooter/ShootPID/Shooter P", P);
        SmartDashboard.putNumber("Shooter/ShootPID/Shooter I", I);
        SmartDashboard.putNumber("Shooter/ShootPID/Shooter D", D); 
        SmartDashboard.putNumber("Shooter/ShootPID/Shooter F", F);
        SmartDashboard.putNumber("Shooter/ShootPID/Shooter Setpoint", 3000);
    }

    public void updateConstants() {
        controller.setP(SmartDashboard.getNumber("Shooter P", P));
        controller.setI(SmartDashboard.getNumber("Shooter I", I));
        controller.setD(SmartDashboard.getNumber("Shooter D", D));
        controller.setFF(SmartDashboard.getNumber("Shooter F", F));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Limelight Distance", getDistance());
        SmartDashboard.putNumber("RPM Setpoint", findRPM());
        publishRPM();
    }
}
