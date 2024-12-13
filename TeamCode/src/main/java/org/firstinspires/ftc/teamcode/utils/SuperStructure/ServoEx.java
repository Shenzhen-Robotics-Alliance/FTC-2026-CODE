package org.firstinspires.ftc.teamcode.utils.SuperStructure;

import com.qualcomm.robotcore.hardware.Servo;

public class ServoEx implements SimpleMechanism {
    private final Servo servo;

    public ServoEx(Servo servo) {
        this.servo = servo;
    }

    @Override
    public void goToPosition(double setPoint) {
        servo.setPosition(setPoint);
    }
}
