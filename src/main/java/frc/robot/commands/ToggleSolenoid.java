package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class ToggleSolenoid extends CommandBase{
    public ToggleSolenoid(){
        addRequirements(Robot.SOLENOID);
      }

      public void initialize() {
      }

      public void execute() {
        Robot.SOLENOID.toggleSolenoid();
      }
    
      public boolean isFinished() {
        return true;
      }
    
      public void end(boolean interrupted) {
      }
}
