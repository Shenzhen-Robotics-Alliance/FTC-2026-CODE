package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.Servo;
@TeleOp(name="test servo")

public class ServoTest extends OpMode {
    private Servo servo1;
    private Servo servo2;

    @Override
    public void init() {
        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo1 = hardwareMap.get(Servo.class, "servo2");

    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            servo1.setPosition(0.5);
            servo2.setPosition(0.5);
        } else if (gamepad1.b) {
            servo1.setPosition(0);
            servo2.setPosition(0);
        }
        else if (gamepad1.x){
            servo1.setPosition(1);
            servo2.setPosition(1);
        }
    }
}