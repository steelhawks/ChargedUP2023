package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotContainer;

import java.util.HashSet;
import java.util.Set;

public class BalanceCommand implements Command 
{
  private PIDController balancer;
  

  public BalanceCommand() {
    balancer = new PIDController(0.05, 0.0038, 0.00595);
    balancer.setSetpoint(-0.7);
    balancer.setTolerance(0.4);
    balancer.enableContinuousInput(0, 360);
  }

  @Override
  public Set<Subsystem> getRequirements() 
  {
    Set<Subsystem> list = new HashSet<Subsystem>();
    list.add(RobotContainer.s_Swerve);
    return list;
  }

  @Override
  public void initialize() {
    System.out.println("STARTED");
  }

  @Override
  public void execute() {
    Translation2d translation = new Translation2d(-balancer.calculate(RobotContainer.s_Swerve.getRoll()), 0);
    RobotContainer.s_Swerve.drive(translation, 0, true, true);
  }
    
  @Override
  public boolean isFinished() {

    //Actual Thing
    // return Math.abs(error) > 0 && Math.abs(error) < 0.75; 
    // return balancer.atSetpoint();
    return false;
  }

  @Override
  public void end(boolean interrupted) {
      RobotContainer.s_Swerve.stop();
  }
}