package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class DetectPiece extends CommandBase {

  private Claw claw;

  public DetectPiece(Claw claw) {
    this.claw = claw;

    addRequirements(claw);
  }

  @Override
  public void execute() {
    this.claw.detectPiece();
  }
}
