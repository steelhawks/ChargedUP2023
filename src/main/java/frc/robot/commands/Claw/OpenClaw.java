// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class OpenClaw extends CommandBase {

  private boolean automatic;

  public OpenClaw(boolean automatic) {
    this.automatic = automatic;

    addRequirements(RobotContainer.s_Claw);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    RobotContainer.s_Claw.openClaw(automatic);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return true;
  }
}
