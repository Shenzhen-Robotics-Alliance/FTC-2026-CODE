package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="testShooter")
public class testShooter extends OpMode {
    private DcMotor shooter;
   // private DcMotor rot;

    @Override
    public void init() {
     //   rot = hardwareMap.get(DcMotor.class,"rot");
        shooter = hardwareMap.get(DcMotor.class,"shooter");
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
      //  rot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      //  rot.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void loop() {
        final double rotPower = gamepad1.left_stick_x;
        //rot.setPower(0.3*rotPower);

        if(gamepad1.a){
            shooter.setPower(1);
        }else if(gamepad1.b){
            shooter.setPower(0);
        }
    }
}
