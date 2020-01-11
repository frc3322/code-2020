package frc.robot.commands;
import frc.robot.subsystems.Shooter;
import static frc.robot.Robot.shooter;
import edu.wpi.first.wpilibj2.command.*;
import java.util.*;

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