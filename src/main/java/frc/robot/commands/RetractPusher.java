// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class RetractPusher extends CommandBase {
  /** Creates a new RetractPusher. */
  public RetractPusher() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.s_Pusher);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    RobotContainer.s_Pusher.retract();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
