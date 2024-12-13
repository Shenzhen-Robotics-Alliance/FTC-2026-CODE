package org.firstinspires.ftc.teamcode.utils.SuperStructure;

import org.firstinspires.ftc.teamcode.constants.SystemConstants;

public class ProfiledMechanism {
    private final SimpleMechanism mechanism;
    private final double speed;
    private double currentPosition, setPoint;
    public ProfiledMechanism(SimpleMechanism mechanism, double movementTime, double initialPosition) {
        this.mechanism = mechanism;
        this.speed = 1 / movementTime;

        this.currentPosition = initialPosition;
        this.setPoint = initialPosition;
    }

    public void update() {
        update(1.0 / SystemConstants.ROBOT_UPDATE_RATE_HZ);
    }

    public void update(double dt) {
        double difference = setPoint - currentPosition,
                deltaPosition = speed * dt;
        if (Math.abs(difference) < deltaPosition)
            currentPosition = setPoint;
        else
            currentPosition += Math.signum(difference) * deltaPosition;

        mechanism.goToPosition(currentPosition);
    }


    public void requestPosition(double setPoint) {
        this.setPoint = setPoint;
    }

    public boolean atReference() {
        return currentPosition == setPoint;
    }
}
