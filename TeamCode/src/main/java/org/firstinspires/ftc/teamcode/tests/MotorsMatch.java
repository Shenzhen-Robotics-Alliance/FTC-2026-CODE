package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MotorsMatch implements SimpleUnitTest {
    private final HardwareMap hardwareMap;
    private final Gamepad gamepad;
    private final DcMotor frontLeft, frontRight, backLeft, backRight;
    private final Telemetry telemetry;

    public MotorsMatch(HardwareMap hardwareMap, Gamepad gamepad, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        this.gamepad = gamepad;
        this.telemetry = telemetry;
    }

    // TODO: match the motors on chassis
    @Override
    public void testStart() {

    }

    @Override
    public void testPeriodic() {
        frontLeft.setPower(gamepad.a ? 0.5:0);
        frontRight.setPower(gamepad.b ? 0.5:0);
        backLeft.setPower(gamepad.x ? 0.5:0);
        backRight.setPower(gamepad.y ? 0.5:0);
        telemetry.addData("front left enc ", frontLeft.getCurrentPosition());
        telemetry.addData("front right enc ", frontRight.getCurrentPosition());
        telemetry.addData("back left enc ", backLeft.getCurrentPosition());
        telemetry.addData("back right enc ", backRight.getCurrentPosition());
    }
}
