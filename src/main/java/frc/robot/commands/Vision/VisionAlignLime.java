// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Vision;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.util.Limelight;
import frc.robot.RobotContainer;

public class VisionAlignLime extends CommandBase {
  
  private int pipeline; 
  
  public VisionAlignLime(int pipeline) {
    addRequirements(RobotContainer.s_Swerve);
    this.pipeline = pipeline; 
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    RobotContainer.s_Vision.GoToTag(pipeline);
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.s_Swerve.stop();
  }

  @Override
  public boolean isFinished() {
    return Limelight.getArea() > 4;
  }
}
