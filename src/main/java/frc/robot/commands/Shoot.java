package frc.robot.commands;

import frc.robot.subsystems.Shooter;
import java.util.Set;
import java.util.HashSet;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import static frc.robot.Robot.shooter;

public class Shoot implements Command {
    @Override
    public Set<Subsystem> getRequirements() {
        Set<Subsystem> requSet = new HashSet<Subsystem>();
        requSet.add(shooter);
        return requSet;

    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){

    }

    @Override
    public boolean isFinished() {
        return false;
    }

}