package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.lib.util.ElevatorLevels;
import frc.robot.RobotContainer;

import java.util.HashSet;
import java.util.Set;

public class ElevatorCommand implements Command {

  private ElevatorLevels level;
  private PIDController setter;

  public ElevatorCommand(ElevatorLevels level) {
    this.level = level;

    setter = new PIDController(0.03, 0, 0);
    setter.setTolerance(1);
    setter.setSetpoint(level.getEncoderVal());
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
    double speed = setter.calculate(RobotContainer.s_Elevator.getEncoderVal(), level.getEncoderVal()); // TODO fix this
    RobotContainer.s_Elevator.moveToPosition(speed);
  }
    
  @Override
  public boolean isFinished() {
    return setter.atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
      RobotContainer.s_Elevator.stop();
  }
}