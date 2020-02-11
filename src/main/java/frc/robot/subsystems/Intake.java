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

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap;

import static frc.robot.Robot.m_can;

public class Intake extends SubsystemBase {
    /**
     * Creates a new Intake.
     */
    private final CANSparkMax[] motors = new CANSparkMax[2];
    private final CANEncoder[] encoders = new CANEncoder[2];
    DoubleSolenoid intakeExtender;

    private final int INTAKE_RIGHT = 0, INTAKE_LEFT = 1;

    public Intake() {
        //intakeExtender = new DoubleSolenoid(RobotMap.PCM.PCM_ID, RobotMap.PCM.INTAKE_EXTEND,
        //        RobotMap.PCM.INTAKE_RETRACT);
        motors[INTAKE_RIGHT] = new CANSparkMax(m_can.INTAKE_RIGHT, MotorType.kBrushless);
        motors[INTAKE_LEFT] = new CANSparkMax(m_can.INTAKE_LEFT, MotorType.kBrushless);
        motors[INTAKE_LEFT].follow(motors[INTAKE_RIGHT]);

        encoders[INTAKE_RIGHT] = motors[INTAKE_RIGHT].getEncoder();
        encoders[INTAKE_LEFT] = motors[INTAKE_LEFT].getEncoder();
    }

    public void begin() {
        start();
        extend();
    }

    public void end() {
        stop();
        retract();
    }

    public void start() {
        // TODO: make this a reasonable value as well
        motors[INTAKE_RIGHT].set(0.2);
    }

    public void outtake() {
        motors[INTAKE_RIGHT].set(-0.5);
    }

    public void stop() {
        motors[INTAKE_RIGHT].set(0);
    }

    public void extend() {
        intakeExtender.set(DoubleSolenoid.Value.kForward);
    }

    public void retract() {
        intakeExtender.set(DoubleSolenoid.Value.kReverse);
    }

    public double getVoltage(final int n) {
        return motors[n].getBusVoltage();
    }

    public double getMotorHeat(final int n) {
        return motors[n].getMotorTemperature();
    }

    public double getOutputCurrent(final int n) {
        return motors[n].getOutputCurrent();
    }

    public double getEncoder(final int n) {
        return encoders[n].getPosition();
    }

    public double getVelocity(final int n) {
        return encoders[n].getVelocity();
    }

    public void toggle() {
        if (isExtended()) {
            retract();
        } else {
            extend();
        }
    }

    public boolean isExtended() {
        return intakeExtender.get() == Value.kForward;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
