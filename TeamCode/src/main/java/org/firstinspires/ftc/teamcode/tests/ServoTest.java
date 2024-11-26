package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoTest extends OpMode {
    private Servo servo;

    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, "TestServo");
    }

    @Override
    public void loop() {
        servo.setPosition(0.5 - 0.5 * gamepad1.left_stick_y);
    }
}
