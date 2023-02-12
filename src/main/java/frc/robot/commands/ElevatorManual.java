package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer;

import java.util.HashSet;
import java.util.Set;

public class ElevatorManual implements Command {

  private boolean moveUp;

  public ElevatorManual(boolean moveUp) {
    this.moveUp = moveUp;
  }

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
    RobotContainer.s_Elevator.moveElevator(moveUp);
  }
    
  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void end(boolean interrupted) {
      RobotContainer.s_Elevator.stop();
  }
}