package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

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
    public void execute() {
        drivetrain.driveDistance(distance);
    }
}