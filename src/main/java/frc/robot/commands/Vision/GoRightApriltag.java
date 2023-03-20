// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Vision;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class GoRightApriltag extends CommandBase {

  public GoRightApriltag() {
    addRequirements(RobotContainer.s_Swerve);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    RobotContainer.s_Vision.goRight();
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.s_Swerve.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
