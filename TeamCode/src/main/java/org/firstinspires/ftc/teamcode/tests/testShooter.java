package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="testShooter")
public class testShooter extends OpMode {
    private DcMotor shooter1;
    private DcMotor shooter2;
   // private DcMotor rot;

    @Override
    public void init() {
     //   rot = hardwareMap.get(DcMotor.class,"rot");
        shooter1 = hardwareMap.get(DcMotor.class,"shooter1"); //port0 shooter1 forward
        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter2 = hardwareMap.get(DcMotor.class,"shooter2");//port1 shooter2 forward
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter2.setDirection(DcMotorSimple.Direction.FORWARD);

    }

    @Override
    public void loop() {
        final double rotPower = gamepad1.left_stick_x;

        if(gamepad1.a){
            shooter1.setPower(1);
            shooter2.setPower(1);
        }else if(gamepad1.b){
            shooter1.setPower(0);
            shooter2.setPower(0);
        }
    }
}
