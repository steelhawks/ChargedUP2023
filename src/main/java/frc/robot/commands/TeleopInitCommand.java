/************************ PROJECT DORCAS ************************/
/* Copyright (c) 2022 StuyPulse Robotics. All rights reserved.  */
/* This work is licensed under the terms of the MIT license.    */
/****************************************************************/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

public class TeleopInitCommand extends InstantCommand {

    private final RobotContainer robot;

    public TeleopInitCommand(RobotContainer robot) {
        this.robot = robot;

        // addRequirements(robot.colorSensor);
        // addRequirements(robot.intake);
        // addRequirements(robot.climber);
        // addRequirements(robot.conveyor);
    }

    @Override
    public void initialize() {
        // robot.intake.stop();

        // robot.colorSensor.getTargetBallUpdate();

        // robot.climber.setTilt(Tilt.NO_TILT);
        // robot.climber.resetEncoder();

        // robot.conveyor.setMode(ConveyorMode.DEFAULT);
        // robot.conveyor.setTopBelt(Direction.STOPPED);
        // robot.conveyor.setGandalf(Direction.STOPPED);
    }
}
