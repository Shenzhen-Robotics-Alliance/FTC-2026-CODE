package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="TestSuperStruct")
public class SingleFileOpMode extends OpMode {
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotorEx elevator;

    private Servo arm1, arm2, wristRot, wristFlip, claw;
    @Override
    public void init() {
        this.frontLeft = hardwareMap.get(DcMotor.class, "FrontLeftMotor");
        this.frontRight = hardwareMap.get(DcMotor.class, "FrontRightMotor");
        this.backLeft = hardwareMap.get(DcMotor.class, "BackLeftMotor");
        this.backRight = hardwareMap.get(DcMotor.class, "BackRightMotor");

        this.frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        this.backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        this.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        this.elevator = hardwareMap.get(DcMotorEx.class, "Elevator");

        this.arm1 = hardwareMap.get(Servo.class, "Arm1");
        this.arm2 = hardwareMap.get(Servo.class, "Arm2");
        this.wristFlip = hardwareMap.get(Servo.class, "WristFlip");
        this.wristRot = hardwareMap.get(Servo.class, "WristRot");
        this.claw = hardwareMap.get(Servo.class, "Claw");
        arm2.setDirection(Servo.Direction.REVERSE);

        this.state = State.HOLD;
        this.closeClaw = true;
    }

    // states
    enum State {
        GRAB,
        HOLD,
        SCORE
    }
    private State state;
    private boolean closeClaw;

    @Override
    public void loop() {
        final double forward = gamepad1.left_stick_y;
        final double strafe = gamepad1.left_stick_x;
        final double omega = gamepad1.right_stick_x;

        frontLeft.setPower(forward + strafe + omega);
        frontRight.setPower(forward - strafe - omega);
        backLeft.setPower(forward - strafe + omega);
        backRight.setPower(forward + strafe - omega);

        FSM();

        // update claw position
        claw.setPosition(closeClaw ? 1: 0);
    }

    // finite state machine
    public void FSM() {
        switch (state) {
            case GRAB: {
                // run close loop on elevator
                elevator.setTargetPosition(0);

                // wrist flip
                wristFlip.setPosition(0);

                // arm
                arm1.setPosition(0);
                arm2.setPosition(0);

                // run wrist rot
                final double wristRotRadians = Math.atan2(gamepad2.left_stick_y, gamepad2.left_stick_x);
                if (Math.hypot(gamepad2.left_stick_y, gamepad2.left_stick_x) > 0.5)
                    wristRot.setPosition(wristRotRadians / Math.PI);
                break;
            }

            case HOLD: {
                // run close loop on elevator
                elevator.setTargetPosition(0);

                // wrist flip
                wristFlip.setPosition(0.5);

                // arm
                arm1.setPosition(0.7);
                arm2.setPosition(0.7);

                // run wrist rot
                wristRot.setPosition(0.5);
                break;
            }

            case SCORE: {
                // run close loop on elevator
                elevator.setTargetPosition(1000);

                // wrist flip
                wristFlip.setPosition(0.5);

                // arm
                arm1.setPosition(0.7);
                arm2.setPosition(0.7);

                // run wrist rot
                wristRot.setPosition(0.5);
            }
        }
    }
}
