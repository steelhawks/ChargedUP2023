/************************ PROJECT DORCAS ************************/
/* Copyright (c) 2022 StuyPulse Robotics. All rights reserved.  */
/* This work is licensed under the terms of the MIT license.    */
/****************************************************************/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DrivetrainDriveCommand extends CommandBase {

    private Drivetrain drivetrain;
    private Joystick driver;

    public DrivetrainDriveCommand(Drivetrain drivetrain, Joystick driver) {
        this.drivetrain = drivetrain;
        this.driver = driver;

        addRequirements(drivetrain);
    }

    public void execute() {
        drivetrain.arcadeDrive(driver);
        drivetrain.shuffleBoard();

    }

    public boolean isFinished() {
        return false;
    }
}
