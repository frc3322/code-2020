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
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RobotMap;

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

    private double P = 0.026;
    private double I = 0.0619;
    private double D = 0.00075;

    public Drivetrain() {
        motors[LEFT_BACK] = new CANSparkMax(RobotMap.CAN.LEFT_BACK_MOTOR, MotorType.kBrushless);
        motors[LEFT_FRONT] = new CANSparkMax(RobotMap.CAN.LEFT_FRONT_MOTOR, MotorType.kBrushless);
        motors[RIGHT_BACK] = new CANSparkMax(RobotMap.CAN.RIGHT_BACK_MOTOR, MotorType.kBrushless);
        motors[RIGHT_FRONT] = new CANSparkMax(RobotMap.CAN.RIGHT_FRONT_MOTOR, MotorType.kBrushless);

        motors[LEFT_BACK].follow(motors[LEFT_FRONT]);
        motors[RIGHT_BACK].follow(motors[RIGHT_FRONT]);

        encoders[LEFT_BACK] = motors[LEFT_BACK].getEncoder();
        encoders[LEFT_FRONT] = motors[LEFT_FRONT].getEncoder();
        encoders[RIGHT_BACK] = motors[RIGHT_BACK].getEncoder();
        encoders[RIGHT_FRONT] = motors[RIGHT_FRONT].getEncoder();

        robotDrive = new DifferentialDrive(motors[LEFT_FRONT], motors[RIGHT_FRONT]);

        PID = new PIDController(P, I, D);

        odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

        PID.disableContinuousInput();
        PID.setTolerance(1);
        SmartDashboard.putNumber("Drivetrain P", P);
        SmartDashboard.putNumber("Drivetrain I", I);
        SmartDashboard.putNumber("Drivetrain D", D);
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
    
    public void resetEncoders() {
        encoders[LEFT_FRONT].setPosition(0);
        encoders[RIGHT_FRONT].setPosition(0);
        encoders[LEFT_BACK].setPosition(0);
        encoders[RIGHT_BACK].setPosition(0);
    }

    public void drive(double speed, double rotation) {
        robotDrive.arcadeDrive(speed, rotation);

    }

    public void getLimelightX() {
        limelightX = tx.getDouble(0.0);
        SmartDashboard.putNumber("Limelight tx", limelightX);
    }

    public void pidDrive(Double speed) {
        limelightX = tx.getDouble(0.0);
        PID.reset();
        SmartDashboard.putNumber("Limelight tx", limelightX);
        drive(speed, PID.calculate(limelightX, 0));
    }

    // returns meters traveled
    public double getLeftEncDistance() {
        return encoders[LEFT_FRONT].getPosition() * (Constants.DriveConstants.WHEEL_DIAMETER_INCHES * .0254) * Math.PI;
    }

    public double getRightEncDistance() {
        return -encoders[RIGHT_FRONT].getPosition() * (Constants.DriveConstants.WHEEL_DIAMETER_INCHES * .0254) * Math.PI;
    }


    // returns meters per second
    public double getLeftEncRate() {
        double RPS = encoders[LEFT_FRONT].getVelocity() / 60;
        return RPS * Constants.DriveConstants.WHEEL_CIRCUMFERENCE_METERS;
    }
    public double getRightEncRate() {
        double RPS = encoders[RIGHT_FRONT].getVelocity() / 60;
        return RPS * Constants.DriveConstants.WHEEL_CIRCUMFERENCE_METERS;
    }

    public boolean onTarget() {
        return PID.atSetpoint();
    }

    public void updateConstants() {
        PID.setP(SmartDashboard.getNumber("Drivetrain P", 0));
        PID.setI(SmartDashboard.getNumber("Drivetrain I", 0));
        PID.setD(SmartDashboard.getNumber("Drivetrain D", 0));
    }

    public void tankDrive(double leftSpeed, double rightSpeed) {
        robotDrive.tankDrive(leftSpeed, rightSpeed);
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getLeftEncRate(), getRightEncRate());
    }

    public double getHeading() {
        return Math.IEEEremainder(navx.getAngle(), 360) * (Constants.DriveConstants.GYRO_REVERSED ? -1.0 : 1.0);
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void tankDriveVolts(double leftVolts, double rightVolts) {
        motors[LEFT_FRONT].setVoltage(leftVolts);
        motors[RIGHT_FRONT].setVoltage(-rightVolts);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("ENC Left Position", encoders[LEFT_FRONT].getPosition());
        SmartDashboard.putNumber("ENC Right Position", encoders[RIGHT_FRONT].getPosition());
        SmartDashboard.putNumber("ENC Rate Left m/s", getLeftEncRate());
        SmartDashboard.putNumber("ENC Rate Right m/s", getRightEncRate());
        SmartDashboard.putNumber("Heading", getHeading());
        SmartDashboard.putNumber("ENC Distance Left m", getLeftEncDistance());
        SmartDashboard.putNumber("ENC Distance Right m", getRightEncDistance());
        updateConstants();
        getLimelightX();
        odometry.update(Rotation2d.fromDegrees(getHeading()), getLeftEncDistance(),
                getRightEncDistance());
    }
}
