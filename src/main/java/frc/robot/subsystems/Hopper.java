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

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Robot.m_can;

public class Hopper extends SubsystemBase {
  /**
   * Creates a new Hopper.
   */
  private CANSparkMax[] motors = new CANSparkMax[2];
  private CANEncoder[] encoders = new CANEncoder[2];

  private int LEFT = 0, RIGHT = 1;


  public Hopper() {
    motors[LEFT] = new CANSparkMax(m_can.LEFT_HOPPER_MOTOR, MotorType.kBrushless);
    motors[RIGHT] = new CANSparkMax(m_can.RIGHT_HOPPER_MOTOR, MotorType.kBrushless);

    motors[RIGHT].follow(motors[LEFT]);

    encoders[LEFT] = motors[LEFT].getEncoder();
    encoders[RIGHT] = motors[RIGHT].getEncoder();
  }

  public void cycle() {
    //TODO: make this a reasonable value instead of this placeholder
    motors[LEFT].set(0.2);
  }

  public void stop() {
    motors[LEFT].set(0);
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
    // This method will be called once per scheduler run
  }
}
