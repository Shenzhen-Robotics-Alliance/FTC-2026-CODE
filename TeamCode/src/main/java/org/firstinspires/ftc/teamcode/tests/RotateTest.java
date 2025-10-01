package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="testRot")
public class RotateTest extends OpMode {
    private DcMotor Rot;

    @Override
    public void init() {
        this.Rot = hardwareMap.get(DcMotor.class,"Rot");
        Rot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        final double rotPower = gamepad2.left_stick_x;
        Rot.setPower(0.3*rotPower);
    }

}
