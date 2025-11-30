package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="testShooter")
public class testShooter extends OpMode {
    private DcMotorEx shooter1;
    private DcMotorEx shooter2;
//    private DcMotor rot;

    @Override
    public void init() {
//        rot = hardwareMap.get(DcMotor.class,"rot");
        shooter1 = hardwareMap.get(DcMotorEx.class,"ShooterMotor"); //port0 shooter1 forward
        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter1.setDirection(DcMotorSimple.Direction.FORWARD);
//        shooter2 = hardwareMap.get(DcMotorEx.class,"ShooterMotor2");//port1 shooter2 forward
//        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        shooter2.setDirection(DcMotorSimple.Direction.FORWARD);

    }

    @Override
    public void loop() {
        final double rotPower = gamepad1.left_stick_x;
        telemetry.addData("velovity",shooter1.getVelocity());


        if(shooter1.getVelocity()<3000){

//            shooter2.setPower(0.7);

            shooter1.getVelocity();
            if (gamepad1.a){
                shooter1.setPower(1);
            }
            else {
                shooter1.setPower(0);}

        }else if(gamepad1.b){
            shooter1.setPower(0);
//            shooter2.setPower(0);
        }
        else {
            shooter1.setPower(0);
        }
    }
}



