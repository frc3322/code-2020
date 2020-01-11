package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.DriveControl;

import static frc.robot.Robot.limelight;

public class Drivetrain extends Subsystem {

    private DifferentialDrive robotDrive;

    private CANSparkMax[] motors = new CANSparkMax[4];
    private CANEncoder[] encoders = new CANEncoder[4];

    private PIDController PID;

    private final int LEFT_BACK = 0, 
                        LEFT_FRONT = 1, 
                        RIGHT_BACK = 2, 
                        RIGHT_FRONT = 3;

    public Drivetrain() {
        motors[LEFT_BACK] = new CANSparkMax(RobotMap.CAN.LEFT_BACK_MOTOR, MotorType.kBrushless);
        motors[LEFT_FRONT] = new CANSparkMax(RobotMap.CAN.LEFT_FRONT_MOTOR, MotorType.kBrushless);
        motors[RIGHT_BACK] = new CANSparkMax(RobotMap.CAN.RIGHT_BACK_MOTOR, MotorType.kBrushless);
        motors[RIGHT_FRONT] = new CANSparkMax(RobotMap.CAN.RIGHT_FRONT_MOTOR, MotorType.kBrushless);

        motors[LEFT_BACK].follow(motors[LEFT_FRONT]);

        encoders[LEFT_BACK] = motors[LEFT_BACK].getEncoder();
        encoders[LEFT_FRONT] = motors[LEFT_FRONT].getEncoder();
        encoders[RIGHT_BACK] = motors[RIGHT_BACK].getEncoder();
        encoders[RIGHT_FRONT] = motors[RIGHT_FRONT].getEncoder();

        robotDrive = new DifferentialDrive(motors[LEFT_FRONT], motors[RIGHT_FRONT]);    
        
        PID = new PIDController(0, 0, 0);

        PID.disableContinuousInput();
        PID.setTolerance(2.5);

        SmartDashboard.putNumber("Drivetrain P", 0);
        SmartDashboard.putNumber("Drivetrain I", 0);
        SmartDashboard.putNumber("Drivetrain D", 0);
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

    public void drive(double speed, double rotation){
        robotDrive.arcadeDrive(speed, rotation);

    }

    public void pidDrive(Double speed) {
        drive(speed, PID.calculate(limelight.getTx(), 0));
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

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new DriveControl());
    }

    @Override
    public void periodic() {
        updateConstants();
    }
}