package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import java.util.HashSet;
import java.util.Set;

public class ToggleElevator extends CommandBase {

  public ToggleElevator() {
    addRequirements(RobotContainer.s_Elevator);
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
  public void end(boolean interrupted) {
  }
}