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

import static frc.robot.Robot.m_can;

public class Climber extends SubsystemBase {
  private CANSparkMax[] motors;
  private CANEncoder[] encoders;
  DoubleSolenoid intakeExtender;
  
  private int RAISE = 0, CLIMB = 1;
  public Climber() {
    motors[RAISE] = new CANSparkMax(m_can.CLIMBER_RAISE, MotorType.kBrushless);
    motors[CLIMB] = new CANSparkMax(m_can.CLIMBER_CLIMB, MotorType.kBrushless);
    encoders[RAISE] = motors[RAISE].getEncoder();
    encoders[CLIMB] = motors[CLIMB].getEncoder();
  }
 //TODO need to have either mechanical hardstops or stops built in to code 
  public void raiseClimber(double speed) {
    motors[RAISE].set(speed);
  }

  public void lowerClimber(double speed) {
    motors[RAISE].set(-speed);
  }

  public void pullWinch(double speed) {

    motors[CLIMB].set(speed);
  }

  public void pushWinch(double speed) {
    motors[CLIMB].set(speed);
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

  public boolean isExtended() {
    return intakeExtender.get() == Value.kForward;
  }

  @Override
  public void periodic() {
  }
}
