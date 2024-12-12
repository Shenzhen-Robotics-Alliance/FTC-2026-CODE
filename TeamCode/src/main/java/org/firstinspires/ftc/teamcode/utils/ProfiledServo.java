package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.constants.SystemConstants;

public class ProfiledServo implements SimpleMechanism {
    private final Servo servo;
    private final double speed;
    private double currentPosition, setPoint;
    public ProfiledServo(Servo servo, double movementTime, double initialPosition) {
        this.servo = servo;
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

        servo.setPosition(currentPosition);
    }

    @Override
    public void requestPosition(double setPoint) {
        this.setPoint = setPoint;
    }

    @Override
    public boolean atReference() {
        return currentPosition == setPoint;
    }
}
