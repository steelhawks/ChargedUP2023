package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer;

import java.util.HashSet;
import java.util.Set;

public class ToggleElevator implements Command {

  public ToggleElevator() {}

  @Override
  public Set<Subsystem> getRequirements() 
  {
    Set<Subsystem> list = new HashSet<Subsystem>();
    list.add(RobotContainer.s_Elevator);
    return list;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    RobotContainer.s_Elevator.togglePistons();
  }
    
  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void end(boolean interrupted) {}
}