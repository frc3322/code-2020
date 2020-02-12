package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Drivetrain.PIDMode;

public class DriveDistance extends CommandBase {

    Drivetrain drivetrain;

    private double distance;

    public DriveDistance(Drivetrain d_subsystem, double distance) {
        drivetrain = d_subsystem;
        addRequirements(drivetrain);

        this.distance = distance;
        drivetrain.driveDistance(distance);
    }

    @Override
    public void initialize(){
        drivetrain.setUpPID(PIDMode.DISTANCE);
        drivetrain.resetEncoders();
    }

    @Override
    public void execute() {
        drivetrain.driveDistance(distance);
    }

    @Override
    public boolean isFinished(){
        return drivetrain.distanceOnTarget(distance);
    }
}