/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.RobotMap;

import static frc.robot.Robot.m_can;

public class Spinner extends SubsystemBase {
  private CANSparkMax motor;
  private DoubleSolenoid spinnerExtender;
  private ColorSensorV3 m_colorSensor;
  private char color = 'U';

  public Spinner() {
    spinnerExtender = new DoubleSolenoid(RobotMap.PCM.SPINNER_UP, RobotMap.PCM.SPINNER_DOWN);
    motor = new CANSparkMax(m_can.SPINNER_MOTOR, MotorType.kBrushless);
    m_colorSensor = new ColorSensorV3(Robot.i2cPort);
  }

  public void spin(double speed) {
    motor.set(speed);
  }

  public void stop() {
    motor.set(0);
  }

  public void extend() {
    spinnerExtender.set(Value.kForward);
  }

  public void retract() {
    spinnerExtender.set(Value.kReverse);
  }

  public void toggle() {
    if (spinnerExtender.get() == Value.kReverse) {
      extend();
    } else retract();
  }

  public char getColor() {
    return color;
  }

  @Override
  public void periodic() {
    Color detectedColor = m_colorSensor.getColor();

    double IR = m_colorSensor.getIR();
    /*
    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("IR", IR);
    */

    // rough thresholds for color detection
    if (detectedColor.red > 0.5) {
      color = 'R';
      SmartDashboard.putString("Color", "Red");
    } else if (detectedColor.green > 0.5) {
      color = 'G';
      SmartDashboard.putString("Color", "Green");
    } else if (detectedColor.blue > 0.3) {
      color = 'B';
      SmartDashboard.putString("Color", "Blue");
    } else if (detectedColor.red > 0.35 && detectedColor.green > 0.35) {
      color = 'Y';
      SmartDashboard.putString("Color", "Yellow");
    } else {
      color = 'U';
      SmartDashboard.putString("Color", "Unsure");
    }
  }
}
