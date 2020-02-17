package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Drivetrain.PIDMode;

public class DriveDistance extends CommandBase {

    Drivetrain drivetrain;

    private double distance;
    private double initAngle;

    public DriveDistance(Drivetrain d_subsystem, double distance) {
        drivetrain = d_subsystem;
        addRequirements(drivetrain);

        this.distance = distance;
    }
    
    @Override
    public void initialize(){
        drivetrain.setUpPID(PIDMode.DISTANCE);
        drivetrain.setUpPID(PIDMode.STRAIGHTEN);
        drivetrain.resetEncoders();

        initAngle = drivetrain.getHeading();
    }

    @Override
    public void execute() {
        drivetrain.driveDistance(distance, initAngle);
    }

    @Override
    public boolean isFinished(){
        return drivetrain.distanceOnTarget(distance);
    }
}