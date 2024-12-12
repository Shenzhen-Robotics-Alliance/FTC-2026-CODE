package org.firstinspires.ftc.teamcode.utils;

/**
 * Represents any 1 DOF mechanisms
 * the position of such mechanism is in the range 0~1
 * */
public interface SimpleMechanism {
    /**
     * Request the linear motion to move to a position
     * */
    void requestPosition(double setPoint);

    /**
     * Whether the linear motion have archived to that position
     * Appropriate tolerance should be given for each specific mechanisms
     * */
    boolean atReference();
}
