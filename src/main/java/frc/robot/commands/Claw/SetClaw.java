// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.util.ClawStatus;
import frc.robot.RobotContainer;

public class SetClaw extends CommandBase {

  private ClawStatus status;

  public SetClaw(ClawStatus status) {
    this.status = status;

    addRequirements(RobotContainer.s_Claw);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (status == ClawStatus.OPEN) {
      RobotContainer.s_Claw.forward();
    } else {
      RobotContainer.s_Claw.reverse();
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return true;
  }
}
