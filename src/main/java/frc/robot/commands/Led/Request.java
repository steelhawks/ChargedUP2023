package frc.robot.commands.Led;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.lib.util.LEDColor;

public class Request extends CommandBase {

    private LEDColor color;

    public Request(LEDColor color) {
        this.color = color;

        addRequirements(RobotContainer.s_Led);
    }

    @Override
    public void initialize() {
        RobotContainer.s_Led.setColor(color);
    }

    @Override
    public void execute() {
        
    }

    @Override
    public boolean isFinished() {
        return !RobotContainer.s_Claw.beamBreaker.get();
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}