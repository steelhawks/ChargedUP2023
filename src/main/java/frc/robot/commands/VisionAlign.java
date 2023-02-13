// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.util.Limelight;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class VisionAlign extends CommandBase {
  
  public VisionAlign() {
    addRequirements(RobotContainer.s_Swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Limelight.hasValidTarget() && Limelight.getArea() < 1.6) {
      double y_vel;
      if (Math.abs(Limelight.getXOffset()) > 1) {
        y_vel = -(Math.sin(Math.PI / 54) * Limelight.getXOffset());
      } else {
        y_vel = 0;
      }

      Translation2d velocity = new Translation2d(0.8, y_vel);
      RobotContainer.s_Swerve.drive(velocity, 0, true, false);
    } else if (!Limelight.hasValidTarget()) {
      RobotContainer.s_Swerve.drive(new Translation2d(0, 0), 1, true, false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.s_Swerve.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Limelight.getArea() > 1.6;
  }
}
