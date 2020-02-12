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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Robot.m_can;

public class Climber extends SubsystemBase {
    private int RAISE = 0, CLIMB = 1;
    private CANSparkMax[] motors = new CANSparkMax[2];
    private CANEncoder[] encoders = new CANEncoder[2];
    DoubleSolenoid intakeExtender;

    public Climber() {
        motors[RAISE] = new CANSparkMax(m_can.CLIMBER_RAISE, MotorType.kBrushless);
        motors[CLIMB] = new CANSparkMax(m_can.CLIMBER_CLIMB, MotorType.kBrushless);
        encoders[RAISE] = motors[RAISE].getEncoder();
        encoders[CLIMB] = motors[CLIMB].getEncoder();
    }

    // TODO make the encoder points at the hardstops actual values
    public void raiseClimber(double speed) {
        if (encoders[RAISE].getPosition() < 2000) {
            motors[RAISE].set(speed);
        } else {
            stopClimber();
        }
    }

    public void lowerClimber(double speed) {
        if (encoders[RAISE].getPosition() > 0) {
            motors[RAISE].set(-speed);
        } else {
            stopClimber();
        }
    }

    public void stopClimber() {
        motors[RAISE].set(0);
    }

    public void pullWinch(double speed) {
        if (encoders[CLIMB].getPosition() > 0) {
            motors[CLIMB].set(speed);
        } else {
            stopWinch();
        }
    }

    public void pushWinch(double speed) {
        if (encoders[CLIMB].getPosition() < 2000) {
            motors[CLIMB].set(speed);
        } else {
            stopWinch();
        }
    }

    public void stopWinch() {
        motors[CLIMB].set(0);
    }

    public void extendHook() {
        intakeExtender.set(DoubleSolenoid.Value.kForward);
    }

    public void retractHook() {
        intakeExtender.set(DoubleSolenoid.Value.kReverse);
    }

    public void toggle() {
        if (isExtended()) {
            retractHook();
        } else {
            extendHook();
        }
    }

    public void setSpeed(double speed) {
        motors[RAISE].set(speed);
    }

    public boolean isExtended() {
        return intakeExtender.get() == Value.kForward;
    }

    public void putInitialDash() {
        SmartDashboard.putNumber("Climb Speed", 0);
    }

    @Override
    public void periodic() {
        //setSpeed(SmartDashboard.getNumber("Climb Speed", 0));
        SmartDashboard.putNumber("Climb Encoder", encoders[RAISE].getPosition());
        //SmartDashboard.putNumber("Voltage Draw Climber", motors[RAISE].getBusVoltage());
        SmartDashboard.putNumber("Output Current Climber", motors[RAISE].getOutputCurrent());
    }
}
