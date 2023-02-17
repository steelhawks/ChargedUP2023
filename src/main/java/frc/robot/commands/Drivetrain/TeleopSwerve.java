package frc.robot.commands.Drivetrain;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class TeleopSwerve extends CommandBase {    
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private IntSupplier POV;
    private PIDController angler;

    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup, IntSupplier POV) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
        this.POV = POV;
        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;

        this.angler = new PIDController(0, 0, 0);
        
    }

    private double calculate(int ang) {
        angler.setSetpoint(ang);
        angler.setTolerance(0.5);
        double per = angler.calculate(s_Swerve.getYaw().getDegrees());
        System.out.println(per);
        return per;
    }

    @Override
    public void execute() {
        // switch(stick.getPOV()) {
        //     case 0: 
        //       RobotContainer.s_Swerve.rotateToAngle(0);
        //       return;
        //     case 90:
        //       RobotContainer.s_Swerve.rotateToAngle(90);
        //       return;
        //     case 180:
        //       RobotContainer.s_Swerve.rotateToAngle(180);   
        //       return;
        //     case 270:
        //       RobotContainer.s_Swerve.rotateToAngle(270);
        //       return;
        // }

        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);
        int pov = POV.getAsInt();
        /* Drive */
        if(pov == -1) {
            s_Swerve.drive(
                new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
                rotationVal * Constants.Swerve.maxAngularVelocity, 
                !robotCentricSup.getAsBoolean(), 
                true
            );
        } else if (pov % 90 == 0){
            // System.out.println("FRICKETY FRACK");
            s_Swerve.drive(
                new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
                calculate(pov) * Constants.Swerve.maxAngularVelocity,
                false,
                true
            );
        }
    }
}