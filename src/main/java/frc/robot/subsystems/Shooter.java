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
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import static frc.robot.Robot.m_can;

public class Shooter extends SubsystemBase {

    private static double P = 0.0006;
    private static double I = 0.0000026;
    private static double D = 100;
    private static double F = 0;

    private CANSparkMax[] motors = new CANSparkMax[2];
    private CANEncoder[] encoders = new CANEncoder[2];

    private final int MOTOR_0 = 0, MOTOR_1 = 1;
    private static SpeedControllerGroup shooter;

    CANPIDController controller;

    public Shooter() {
        motors[MOTOR_0] = new CANSparkMax(m_can.SHOOTER_1, MotorType.kBrushless);
        motors[MOTOR_1] = new CANSparkMax(m_can.SHOOTER_2, MotorType.kBrushless);
        motors[MOTOR_0].setInverted(true);
        motors[MOTOR_1].setInverted(false);

        shooter = new SpeedControllerGroup(motors[MOTOR_0], motors[MOTOR_1]);

        encoders[MOTOR_0] = new CANEncoder(motors[MOTOR_0]);
        encoders[MOTOR_1] = new CANEncoder(motors[MOTOR_1]);


        for (CANSparkMax motor : motors) {
            motor.setIdleMode(IdleMode.kCoast);
        }

        controller = motors[MOTOR_0].getPIDController();

        SmartDashboard.putNumber("P", P);
        SmartDashboard.putNumber("I", I);
        SmartDashboard.putNumber("D", D);
        SmartDashboard.putNumber("F", F);

        SmartDashboard.putNumber("Shooter Speed", 0);
        SmartDashboard.putNumber("Set Shooter RPM", 3000);

        controller.setOutputRange(0, 1);
    }

    public void setSetpoint(double setpoint) {
        controller.setReference(setpoint, ControlType.kVelocity);
    }

    public double publishRPM() {
        SmartDashboard.putNumber("Shooter RPM", encoders[MOTOR_0].getVelocity());
        return encoders[MOTOR_0].getVelocity();
    }

    public void setSpeed(double speed) {
        motors[MOTOR_0].set(speed);
    }

    public void updateConstants() {
        setSetpoint(SmartDashboard.getNumber("Set Shooter RPM", 3000));
        controller.setP(SmartDashboard.getNumber("Shooter P", 0));
        controller.setI(SmartDashboard.getNumber("Shooter I", 0));
        controller.setD(SmartDashboard.getNumber("Shooter D", 0));
        controller.setFF(SmartDashboard.getNumber("Shooter F", 0));
    }

    @Override
    public void periodic() {
        //setSpeed(SmartDashboard.getNumber("Shooter Speed", 0));
        updateConstants();
        publishRPM();
    }
}
