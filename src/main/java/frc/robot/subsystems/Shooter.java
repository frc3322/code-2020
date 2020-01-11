package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.Shoot;

public class Shooter implements Subsystem {

    private static double P = 0.03;
    private static double I = 0;
    private static double D = 0;
    private static double F = 4;

    private CANSparkMax[] motors = new CANSparkMax[2];
    private CANEncoder[] encoders = new CANEncoder[2];

    private final int MOTOR_0 = 0,
                        MOTOR_1 = 1;

    private final int ENCODER_0 = 0,
                        ENCODER_1 = 1;

    public Shooter() {
        //super("Shooter PID", P, I, D, F);
        CommandScheduler.getInstance().registerSubsystem(this);
        motors[MOTOR_0] = new CANSparkMax(RobotMap.CAN.SHOOTER_1, MotorType.kBrushless);
        motors[MOTOR_1] = new CANSparkMax(RobotMap.CAN.SHOOTER_2, MotorType.kBrushless);

        encoders[ENCODER_0] = new CANEncoder(motors[MOTOR_0]);
        encoders[ENCODER_1] = new CANEncoder(motors[MOTOR_1]);

        motors[MOTOR_1].follow(motors[MOTOR_0]);
        /*
        setAbsoluteTolerance(20);
        getPIDController().setContinuous(false);
        setOutputRange(0, 1);
        */
    }
    /*
    @Override
    public void setSetpoint(double setpoint) {
        super.setSetpoint(setpoint);
    }

    @Override
    protected double returnPIDInput() {
        return encoders[ENCODER_0].getVelocity();
    }

    @Override
    protected void usePIDOutput(double output) {
        SmartDashboard.putNumber("PID Output", output);
        motors[MOTOR_0].pidWrite(output);
    }
    */
    @Override
    public void setDefaultCommand(Command defaultCommand) {
        defaultCommand = new Shoot();
        CommandScheduler.getInstance().setDefaultCommand(this, defaultCommand);
    }


}