package org.firstinspires.ftc.teamcode.tests;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@TeleOp(name = "test")
public class SimpleMovementTest extends OpMode {
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private double kP =0 ;
    private double kI = 0;
    private double kD = 0;
    private PIDController pidController = new PIDController(kP, kI, kD);

    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft"); // FrontLeft
        frontRight = hardwareMap.get(DcMotor.class, "frontRight"); // frontRight reverse need
        backLeft = hardwareMap.get(DcMotor.class, "backLeft"); // backleft reverse need
        backRight = hardwareMap.get(DcMotor.class, "backRight"); // backright

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void loop() {
        double speed = 0.3;
        if(gamepad1.dpad_up){
            frontLeft.setPower(speed);
            frontRight.setPower(speed);
            backLeft.setPower(speed);
            backRight.setPower(speed);
        } else if (gamepad1.dpad_left) {
            frontLeft.setPower(-speed);
            frontRight.setPower(speed);
            backLeft.setPower(speed);
            backRight.setPower(-speed);
        }
        else if (gamepad1.dpad_right) {
            frontLeft.setPower(speed);
            frontRight.setPower(-speed);
            backLeft.setPower(-speed);
            backRight.setPower(speed);
        }
        else if (gamepad1.dpad_down) {
            frontLeft.setPower(-speed);
            frontRight.setPower(-speed);
            backLeft.setPower(-speed);
            backRight.setPower(-speed);
        }

    }
}
