package frc.lib.util;

import frc.robot.Constants;

public enum ElevatorLevels {
    HOME(Constants.Elevator.homeEncoderPos),
    LOW(Constants.Elevator.lowEncoderPos),
    MID(Constants.Elevator.midEncoderPos),
    HIGH(Constants.Elevator.highEncoderPos);

    private double encoderVal;

    ElevatorLevels(double encoderVal) {
        this.encoderVal = encoderVal;
    }

    public double getEncoderVal() {
        return encoderVal;
    }
}
