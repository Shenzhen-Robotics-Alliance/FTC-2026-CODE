package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="testServoRot")
public class testServoRotate extends OpMode {

    private Servo servo;
    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class,"servo");
    }

    @Override
    public void loop() {
        double rotateValue = gamepad1.left_stick_x;
        servo.setPosition(rotateValue);
    }
}
