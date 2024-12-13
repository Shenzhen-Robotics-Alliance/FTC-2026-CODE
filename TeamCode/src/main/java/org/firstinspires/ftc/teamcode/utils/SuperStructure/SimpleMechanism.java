package org.firstinspires.ftc.teamcode.utils.SuperStructure;

/**
 * Represents any 1 DOF mechanisms
 * the position of such mechanism is in the range 0~1
 * */
public interface SimpleMechanism {
    /**
     * Request the linear motion to move to a position
     * */
    void goToPosition(double setPoint);
}
