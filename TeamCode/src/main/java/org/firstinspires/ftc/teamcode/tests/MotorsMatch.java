package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "MotorsMatch")
public class MotorsMatch extends OpMode {
    private DcMotor mot0, mot1, mot2, mot3;

    @Override
    public void init() {
        mot0 = hardwareMap.get(DcMotor.class, "mot0"); // fl, n; rig, r
        mot1 = hardwareMap.get(DcMotor.class, "mot1"); // bl, n; hor, r
        mot2 = hardwareMap.get(DcMotor.class, "mot2"); // br, r; lef, n
        mot3 = hardwareMap.get(DcMotor.class, "mot3"); // fr, r
    }

    @Override
    public void loop() {
        mot0.setPower(gamepad1.a ? 0.5:0);
        mot1.setPower(gamepad1.b ? 0.5:0);
        mot2.setPower(gamepad1.x ? 0.5:0);
        mot3.setPower(gamepad1.y ? 0.5:0);
        telemetry.addData("mot 0 enc ", mot0.getCurrentPosition());
        telemetry.addData("mot 1 enc ", mot1.getCurrentPosition());
        telemetry.addData("mot 2 enc ", mot2.getCurrentPosition());
        telemetry.addData("mot 3 enc ", mot3.getCurrentPosition());
    }
}
