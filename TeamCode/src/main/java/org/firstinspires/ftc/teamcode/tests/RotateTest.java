package org.firstinspires.ftc.teamcode.tests;

import static com.arcrobotics.ftclib.hardware.motors.Motor.Direction.FORWARD;
import static com.arcrobotics.ftclib.hardware.motors.Motor.Direction.REVERSE;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "FTCLib RotTest", group = "Test")
public class RotateTest extends LinearOpMode {


    private Motor Rot;

    @Override
    public void runOpMode() throws InterruptedException {

        Rot = new Motor(hardwareMap, "Rot");


        Rot.setRunMode(Motor.RunMode.PositionControl);


        Rot.setPositionCoefficient(0.05);
        Rot.setPositionTolerance(100);

        Rot.setInverted(false);

        Rot.resetEncoder();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            int targetPos = 1000;
            Rot.setTargetPosition(targetPos);


            Rot.set(0.01);

            while (opModeIsActive()) {


                if (Rot.atTargetPosition()) {
                    telemetry.addData("Status", "Target Reached!");

                } else {
                    telemetry.addData("Status", "Moving to Target...");
                }


                telemetry.addData("Current Position", Rot.getCurrentPosition());
                telemetry.addData("Target Position", targetPos);
                telemetry.update();
            }
            Rot.stopMotor();
        }
    }
}