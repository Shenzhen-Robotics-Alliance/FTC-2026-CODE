// package org.firstinspires.ftc.teamcode;
package org.firstinspires.ftc.teamcode.tests;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

// @TeleOp(name="TestSuperStruct")
public class SingleFileOpMode extends OpMode {
    private DcMotor FrontLeftMotor, FrontRightMotor, BackLeftMotor, BackRightMotor;
    private DcMotorEx elevator;

    private Servo wristRot, wristFlip, claw, arm1, arm2;
    @Override
    public void init() {
        this.FrontLeftMotor  = hardwareMap.get(DcMotor.class, "FrontLeftMotor");
        this.FrontRightMotor = hardwareMap.get(DcMotor.class, "FrontRightMotor");
        this.BackLeftMotor = hardwareMap.get(DcMotor.class, "BackLeftMotor");
        this.BackRightMotor = hardwareMap.get(DcMotor.class, "BackRightMotor");

        this.FrontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.BackLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.FrontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.FrontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.BackLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.BackRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.elevator = hardwareMap.get(DcMotorEx.class, "Elevator");

        this.arm1 = hardwareMap.get(Servo.class, "Arm1");
        this.arm2 = hardwareMap.get(Servo.class, "Arm2");
        arm2.setDirection(Servo.Direction.REVERSE);
        this.wristFlip = hardwareMap.get(Servo.class, "WristFlip");
        this.wristRot = hardwareMap.get(Servo.class, "WristRot");
        this.claw = hardwareMap.get(Servo.class, "Claw");

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

        FrontLeftMotor.setPower(forward - strafe + omega);
        FrontRightMotor.setPower(forward + strafe - omega);
        BackLeftMotor.setPower(forward + strafe + omega);
        BackRightMotor.setPower(forward - strafe - omega);


        FSM();

        // update claw position
        claw.setPosition(closeClaw ? 0.7: 0);
        elevator.setPower(gamepad2.right_stick_y);
        telemetry.addData("elevator position", elevator.getCurrentPosition());
    }

    // finite state machine
    public void FSM() {
        if(gamepad2.a) {
            state = State.GRAB;
        }
        else if(gamepad2.b) {
            closeClaw = true;
            state = State.HOLD;
        }
        else if(gamepad2.x){
            state = State.SCORE;
        }
        if (state!= State.HOLD) {
            if(gamepad2.right_trigger > 0.5){
                closeClaw = true;
            }else if(gamepad2.left_trigger > 0.5){
                closeClaw = false;
            }
        }

        switch (state) {
            case GRAB: {
                // run close loop on elevator


                final boolean dropToGround = gamepad2.right_bumper;

                // wrist flip
                wristFlip.setPosition(dropToGround ? 0.1:0);

                // arm
                arm1.setPosition(dropToGround ? 0.2:0.3);
                arm2.setPosition(dropToGround ? 0.2:0.3);

                // run wrist rot
                wristRot.setPosition(0.5 + 0.5 * gamepad2.left_stick_x);
                telemetry.addData("wrist rot", 0.5 + 0.5 * gamepad2.left_stick_x);

                break;
            }

            case HOLD: {
                // run close loop on elevator

                // wrist flip
                wristFlip.setPosition(1);

                // arm
                arm1.setPosition(0.75);
                arm2.setPosition(0.75);

                // run wrist rot
                wristRot.setPosition(0.5);
                break;
            }

            case SCORE: {
                // run close loop on elevator

                // wrist flip
                wristFlip.setPosition(1);

                // arm
                arm1.setPosition(0.8);
                arm2.setPosition(0.8);

                // run wrist rot
                wristRot.setPosition(0.5);

                break;
            }
        }
    }
}