package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class OpenClaw extends CommandBase {

  private Claw claw;

  public OpenClaw(Claw claw) {
    this.claw = claw;

    addRequirements(claw);
  }

  @Override
  public void execute() {
    this.claw.openClaw();
  }
}
