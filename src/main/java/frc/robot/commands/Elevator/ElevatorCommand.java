package frc.robot.commands.Elevator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.lib.util.ElevatorLevels;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import java.util.HashSet;
import java.util.Set;

public class ElevatorCommand extends CommandBase {

  private ElevatorLevels level;
  private PIDController setter;

  public ElevatorCommand(ElevatorLevels level) {
    this.level = level;

    setter = new PIDController(Constants.Elevator.kP, Constants.Elevator.kI, Constants.Elevator.kD);
    setter.setTolerance(Constants.Elevator.tolerance);
    setter.setSetpoint(level.getEncoderVal());

    addRequirements(RobotContainer.s_Elevator);
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
    double speed = -setter.calculate(RobotContainer.s_Elevator.getEncoderRotations(), level.getEncoderVal());

    if (level == ElevatorLevels.HOME) {
      if(RobotContainer.s_Elevator.getEncoderRotations() > Constants.Elevator.minPivotEncoderPos + Constants.Elevator.minPivotTolerance) {
        speed = Constants.Elevator.homeSpeed;
      }
      else {
        speed = Constants.Elevator.homeLowSpeed;
      }
    }

    if (level == ElevatorLevels.DOUBLE_STATION) {
      RobotContainer.s_Elevator.moveElevator(speed, false, true);
    }
    else {
      RobotContainer.s_Elevator.moveElevator(speed, false, false);
    }
  }
    
  @Override
  public boolean isFinished() {
    if (level == ElevatorLevels.HOME) {
      return RobotContainer.s_Elevator.limitLowPressed();
    }

    return setter.atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
      RobotContainer.s_Elevator.stop();
  }
}