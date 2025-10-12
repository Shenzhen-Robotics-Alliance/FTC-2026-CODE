package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="testIntake")
public class intakeTest extends OpMode {
    private DcMotor intake;


    @Override
    public void init() {
        this.intake = hardwareMap.get(DcMotor.class,"intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        if(gamepad2.b){
            intake.setPower(0.8);
        }else if (gamepad2.a){
            intake.setPower(-0.8);
        }else if(gamepad2.y){
            intake.setPower(0);
        }
    }
}
