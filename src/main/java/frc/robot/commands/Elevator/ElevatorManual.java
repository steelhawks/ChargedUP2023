package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import java.util.HashSet;
import java.util.Set;

public class ElevatorManual extends CommandBase {

  private boolean moveUp;

  public ElevatorManual(boolean moveUp) {
    this.moveUp = moveUp;

    addRequirements(RobotContainer.s_Elevator);
  }


  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double speed = moveUp ? -1 * Constants.Elevator.elevatorSpeed : Constants.Elevator.elevatorSpeed;
    System.out.println("Elevator");
    RobotContainer.s_Elevator.moveElevator(speed, true);
  }
    
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
      RobotContainer.s_Elevator.stop();
  }
}