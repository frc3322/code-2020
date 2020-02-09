/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Robot.m_can;

public class Drivetrain extends SubsystemBase {
    private DifferentialDrive robotDrive;

    private CANSparkMax[] motors = new CANSparkMax[4];
    private CANEncoder[] encoders = new CANEncoder[4];

    private AHRS navx = new AHRS(SPI.Port.kMXP);

    private PIDController PID;

    private DifferentialDriveOdometry odometry;

    private final int LEFT_BACK = 0, LEFT_FRONT = 1, RIGHT_BACK = 2, RIGHT_FRONT = 3;

    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    private NetworkTableEntry tx = table.getEntry("tx");
    private NetworkTableEntry ty = table.getEntry("ty");
    private NetworkTableEntry ta = table.getEntry("ta");

    private double limelightX = tx.getDouble(0.0);
    private double limelightY = ty.getDouble(0.0);
    private double limelightA = ta.getDouble(0.0);

    private double P = 0.01893;
    private double I = 1.3;
    private double D = 0.002;

    private double dP = 0;
    private double dI = 0;
    private double dD = 0;

    public enum PIDMode {
      LIMELIGHT,
      ANGLE,
      DISTANCE
    }

    public Drivetrain() {
        motors[LEFT_BACK] = new CANSparkMax(m_can.LEFT_BACK_MOTOR, MotorType.kBrushless);
        motors[LEFT_FRONT] = new CANSparkMax(m_can.LEFT_FRONT_MOTOR, MotorType.kBrushless);
        motors[RIGHT_BACK] = new CANSparkMax(m_can.RIGHT_BACK_MOTOR, MotorType.kBrushless);
        motors[RIGHT_FRONT] = new CANSparkMax(m_can.RIGHT_FRONT_MOTOR, MotorType.kBrushless);

        motors[LEFT_FRONT].setInverted(true);
        motors[RIGHT_FRONT].setInverted(true);

        motors[LEFT_BACK].follow(motors[LEFT_FRONT]);
        motors[RIGHT_BACK].follow(motors[RIGHT_FRONT]);

        motors[LEFT_FRONT].setIdleMode(IdleMode.kBrake);
        motors[LEFT_BACK].setIdleMode(IdleMode.kBrake);
        motors[RIGHT_FRONT].setIdleMode(IdleMode.kBrake);
        motors[RIGHT_BACK].setIdleMode(IdleMode.kBrake);

        encoders[LEFT_BACK] = motors[LEFT_BACK].getEncoder();
        encoders[LEFT_FRONT] = motors[LEFT_FRONT].getEncoder();
        encoders[RIGHT_BACK] = motors[RIGHT_BACK].getEncoder();
        encoders[RIGHT_FRONT] = motors[RIGHT_FRONT].getEncoder();

        robotDrive = new DifferentialDrive(motors[LEFT_FRONT], motors[RIGHT_FRONT]);

        PID = new PIDController(P, I, D);
        PID.disableContinuousInput();

        SmartDashboard.putNumber("Drivetrain P", P);
        SmartDashboard.putNumber("Drivetrain I", I);
        SmartDashboard.putNumber("Drivetrain D", D);

        SmartDashboard.putNumber("Distance P", dP);
        SmartDashboard.putNumber("Distance I", dI);
        SmartDashboard.putNumber("Distance D", dD);

        odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

        
    }

    //Motor methods
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
    
    //Driving methods
    public void drive(double speed, double rotation) {
      robotDrive.arcadeDrive(speed, rotation);
    }
    
    public void tankDrive(double leftSpeed, double rightSpeed) {
      robotDrive.tankDrive(leftSpeed, rightSpeed);
    }
    
    //PID Control Methods
    public void setUpPID(PIDMode mode){
      switch(mode){
        case LIMELIGHT:
          PID.reset();
          PID.setP(SmartDashboard.getNumber("Drivetrain P", P));
          PID.setI(SmartDashboard.getNumber("Drivetrain I", I));
          PID.setD(SmartDashboard.getNumber("Drivetrain D", D));
          PID.setTolerance(1);
          break;
        case ANGLE:
          PID.reset();
          PID.setP(SmartDashboard.getNumber("Drivetrain P", P));
          PID.setI(SmartDashboard.getNumber("Drivetrain I", I));
          PID.setD(SmartDashboard.getNumber("Drivetrain D", D));
          PID.setTolerance(1);
          break;
        case DISTANCE:
          PID.reset();
          PID.setP(SmartDashboard.getNumber("Distance P", dP));
          PID.setI(SmartDashboard.getNumber("Distance I", dI));
          PID.setD(SmartDashboard.getNumber("Distance D", dD));
          PID.setTolerance(1);
          break;
        default:
          PID.reset();
          PID.setP(SmartDashboard.getNumber("Drivetrain P", P));
          PID.setI(SmartDashboard.getNumber("Drivetrain I", I));
          PID.setD(SmartDashboard.getNumber("Drivetrain D", D));
          PID.setTolerance(1);
          break;
      }    
    }

    //Limelight PID methods
    public void getLimelightX() {
        limelightX = tx.getDouble(0.0);
        SmartDashboard.putNumber("Limelight tx", limelightX);
    }

    public void getLimelightY() {
        limelightY = ty.getDouble(0.0);
        SmartDashboard.putNumber("Limelight ty", limelightY);
    }

    public boolean limelightOnTarget() {
        return Math.abs(limelightX) < 1.7;
    }
    
    public void pidDrive(Double speed) {
        limelightX = tx.getDouble(0.0);
        SmartDashboard.putNumber("Limelight tx", limelightX);
        drive(speed, PID.calculate(limelightX, 0));
    }

    //Other PID Drive methods
    public void turnToAngle(double angle) {
        drive(0, PID.calculate(navx.getFusedHeading(), angle));
    }

    public Boolean angleOnTarget(double angle) {
        return Math.abs(angle - navx.getFusedHeading()) < 1;
    }

    public void driveDistance(double distance) {
        double avgDist = (encoders[LEFT_FRONT].getPosition() + encoders[RIGHT_FRONT].getPosition()) / 2;
        drive(PID.calculate(avgDist, distance), 0);
    }

    public Boolean distanceOnTarget(double distance) {
        double avgDist = (encoders[LEFT_FRONT].getPosition() + encoders[RIGHT_FRONT].getPosition()) / 2;
        return Math.abs(distance - avgDist) < 30;
    }

    //Methods for Auton
    public void resetForAuto() {
      encoders[LEFT_FRONT].setPosition(0);
      encoders[RIGHT_FRONT].setPosition(0);
      encoders[LEFT_BACK].setPosition(0);
      encoders[RIGHT_BACK].setPosition(0);
      navx.zeroYaw();

      Rotation2d originRotation = new Rotation2d(0.0);
      Pose2d originPose = new Pose2d(0.0, 0.0, originRotation);
      odometry.resetPosition(originPose, originRotation);
    }

    // returns meters traveled
    public double getLeftEncDistance() {
        return encoders[LEFT_FRONT].getPosition() * Constants.DriveConstants.WHEEL_CIRCUMFERENCE_METERS * Constants.DriveConstants.GEARING;
    }

    public double getRightEncDistance() {
        return -encoders[RIGHT_FRONT].getPosition() * Constants.DriveConstants.WHEEL_CIRCUMFERENCE_METERS * Constants.DriveConstants.GEARING;
    }

    // returns meters per second
    public double getLeftEncRate() {
        double RPS = (encoders[LEFT_FRONT].getVelocity() * Constants.DriveConstants.GEARING)/ 60;
        return RPS * Constants.DriveConstants.WHEEL_CIRCUMFERENCE_METERS;
    }
    public double getRightEncRate() {
        double RPS = (encoders[RIGHT_FRONT].getVelocity() * Constants.DriveConstants.GEARING) / 60;
        return RPS * Constants.DriveConstants.WHEEL_CIRCUMFERENCE_METERS;
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getLeftEncRate(), getRightEncRate());
    }

    public double getHeading() {
        return -1*Math.IEEEremainder(navx.getAngle(), 360) * (Constants.DriveConstants.GYRO_REVERSED ? -1.0 : 1.0);
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void tankDriveVolts(double leftVolts, double rightVolts) {
        robotDrive.feed();
        motors[LEFT_FRONT].setVoltage(leftVolts);
        motors[RIGHT_FRONT].setVoltage(-rightVolts);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Heading", getHeading());
        SmartDashboard.putNumber("ENC Distance Left m", getLeftEncDistance());
        SmartDashboard.putNumber("ENC Distance Right m", getRightEncDistance());
        SmartDashboard.putString("pose", getPose().toString());
        Translation2d driveTranslation = getPose().getTranslation();
        double driveX = driveTranslation.getX();
        double driveY = driveTranslation.getY();
        SmartDashboard.putNumber("PoseX", driveX);
        SmartDashboard.putNumber("PoseY", driveX);

        SmartDashboard.putBoolean("Limelight on Target?", limelightOnTarget());
        getLimelightX();
        getLimelightY();
        odometry.update(Rotation2d.fromDegrees(getHeading()), getLeftEncDistance(),
                getRightEncDistance());
            
        var translation = odometry.getPoseMeters().getTranslation();

        SmartDashboard.putNumber("Odometry X", translation.getX());
        SmartDashboard.putNumber("Odometry Y", translation.getY());
    }
}
