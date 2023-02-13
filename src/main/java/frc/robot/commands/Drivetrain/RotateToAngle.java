// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class RotateToAngle extends CommandBase {
  
  private int angle;

  public RotateToAngle(int angle) {
    this.angle = angle;
    addRequirements(RobotContainer.s_Swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.s_Swerve.rotateToAngle(angle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.s_Swerve.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double degrees = RobotContainer.s_Swerve.getYaw().getDegrees();
    double setpoint = RobotContainer.s_Swerve.getTargetAngle(angle);

    return (degrees - setpoint > 2 && setpoint - degrees < 2);
  }
}
