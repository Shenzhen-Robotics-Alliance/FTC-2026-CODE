// package org.firstinspires.ftc.teamcode;
package org.firstinspires.ftc.teamcode.tests;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

// @TeleOp(name="TestSuperStruct")
public class SingleFileOpMode extends OpMode {
    private DcMotor FrontLeftMotor, FrontRightMotor, BackLeftMotor, BackRightMotor;
    private DcMotor intake,shooter0,shooter1,shooter2;
    private State state;
    private final double intakeSpeed = 0.8;
    private final double shootSpeed = 0.9;
    private final double outputSpeed = -0.8;
    @Override
    public void init() {
        this.FrontLeftMotor  = hardwareMap.get(DcMotor.class, "FrontLeftMotor");
        this.FrontRightMotor = hardwareMap.get(DcMotor.class, "FrontRightMotor");
        this.BackLeftMotor = hardwareMap.get(DcMotor.class, "BackLeftMotor");
        this.BackRightMotor = hardwareMap.get(DcMotor.class, "BackRightMotor");

        this.FrontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.BackLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.FrontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.FrontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.BackLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.BackRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.shooter0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    // states
    enum State {
        INTAKE,
        HOLD,
        OUTPUT,
        SHOOT
    }

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

    }

    // finite state machine
    public void FSM() {
        if(gamepad1.a) {
            state = State.INTAKE;
        }
        else if(gamepad1.b) {
            state = State.HOLD;
        }
        else if(gamepad1.x) {
            state = State.SHOOT;
        }else if(gamepad1.y){
            state = State.OUTPUT;
        }


        switch (state) {
            case INTAKE: {
                // run close loop on intake
                intake.setPower(intakeSpeed);

                break;
            }

            case HOLD: {
              //stop the close loop
              intake.setPower(0);
              shooter0.setPower(0);
              shooter1.setPower(0);
              shooter2.setPower(0);

              break;
            }

            case SHOOT: {
                // run close loop on SHOOT
                shooter0.setPower(shootSpeed);
                shooter1.setPower(shootSpeed);
                shooter2.setPower(shootSpeed);

                break;
            }

            case OUTPUT: {
                //run close loop on output
                shooter0.setPower(outputSpeed);
                shooter1.setPower(outputSpeed);
                shooter2.setPower(outputSpeed);
            }

        }
    }
}