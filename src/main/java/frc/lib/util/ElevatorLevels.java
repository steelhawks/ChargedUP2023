package frc.lib.util;

import frc.robot.Constants;

public enum ElevatorLevels {
    LOW(Constants.Elevator.lowEncoderPos),
    MID(Constants.Elevator.midEncoderPos),
    HIGH(Constants.Elevator.highEncoderPos);

    private int encoderVal;

    ElevatorLevels(int encoderVal) {
        this.encoderVal = encoderVal;
    }

    public int getEncoderVal() {
        return encoderVal;
    }
}
