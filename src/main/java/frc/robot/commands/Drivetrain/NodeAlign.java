package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.lib.util.AlignType;
import frc.lib.util.Limelight;
import frc.robot.RobotContainer;

import java.util.HashSet;
import java.util.Set;

public class NodeAlign implements Command {

  private PIDController controller;
  private AlignType piece;

  public NodeAlign(AlignType piece) {

    this.piece = piece;

    if (piece == AlignType.CONE) {
      controller = new PIDController(0.1, 0, 0); // 0.15, 0, 0.001
      controller.setTolerance(0.7);
    }
    else {
      controller = new PIDController(0.06, 0, 0.0);
      controller.setTolerance(1);
    }
    controller.setSetpoint(0);
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
    if (piece == AlignType.CUBE) {
      Limelight.setPipeline(0);
    }
    else {
      Limelight.setPipeline(1);
    }
  }

  @Override
  public void execute() {
    Translation2d translation = new Translation2d(0, -controller.calculate(Limelight.getXOffset()));
    RobotContainer.s_Swerve.drive(translation, 0, false, false);
  }
    
  @Override
  public boolean isFinished() {
    //System.out.println("aligned!!!!!!");
    //return Limelight.hasValidTarget() && controller.atSetpoint();
    return false || controller.atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
      RobotContainer.s_Swerve.stop();
  }
}