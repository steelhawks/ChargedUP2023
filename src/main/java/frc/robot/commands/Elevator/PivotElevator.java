package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.lib.util.ElevatorPivot;
import frc.robot.RobotContainer;

import java.util.HashSet;
import java.util.Set;

public class PivotElevator extends CommandBase {

  private ElevatorPivot pivot;

  public PivotElevator(ElevatorPivot pivot) {
    this.pivot = pivot;
    addRequirements(RobotContainer.s_Elevator);
  }


  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (pivot == ElevatorPivot.DOWN) {
      RobotContainer.s_Elevator.reverse();
    }
    else {
      RobotContainer.s_Elevator.forward();
    }
  }
    
  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void end(boolean interrupted) {
  }
}