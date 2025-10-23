package org.firstinspires.ftc.teamcode.tests;

import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.xyzOrientation;
import static org.firstinspires.ftc.teamcode.constants.DriveTrainConstants.CENTER_ODOMETER_WHEEL_INVERTED;
import static org.firstinspires.ftc.teamcode.constants.DriveTrainConstants.CENTER_ODOMETER_WHEEL_NAME;
import static org.firstinspires.ftc.teamcode.constants.DriveTrainConstants.IMU_PARAMS;
import static org.firstinspires.ftc.teamcode.constants.DriveTrainConstants.LEFT_ODOMETER_WHEEL_INVERTED;
import static org.firstinspires.ftc.teamcode.constants.DriveTrainConstants.LEFT_ODOMETER_WHEEL_NAME;
import static org.firstinspires.ftc.teamcode.constants.DriveTrainConstants.ODOMETER_ENCODER_TICKS_PER_REVOLUTION;
import static org.firstinspires.ftc.teamcode.constants.DriveTrainConstants.ODOMETER_WHEELS_RADIUS_METERS;
import static org.firstinspires.ftc.teamcode.constants.DriveTrainConstants.RIGHT_ODOMETER_WHEEL_INVERTED;
import static org.firstinspires.ftc.teamcode.constants.DriveTrainConstants.RIGHT_ODOMETER_WHEEL_NAME;
import static org.firstinspires.ftc.teamcode.constants.DriveTrainConstants.FRONT_LEFT_MOTOR_DIRECTION;
import static org.firstinspires.ftc.teamcode.constants.DriveTrainConstants.FRONT_RIGHT_MOTOR_DIRECTION;
import static org.firstinspires.ftc.teamcode.constants.DriveTrainConstants.BACK_LEFT_MOTOR_DIRECTION;
import static org.firstinspires.ftc.teamcode.constants.DriveTrainConstants.BACK_RIGHT_MOTOR_DIRECTION;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.utils.MapleOdometerWheels.MapleEncoder;

import java.util.Base64;

@TeleOp(name = "MotorsMatch")
public class SimpleMovementTest extends LinearOpMode {
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    IMU imu;

    private double kP = 0;
    private double kI = 0;
    private double kD = 0;
    private double kF = 0;
    double xRotation = 0;  // enter the desired X rotation angle here.
    double yRotation = 0;  // enter the desired Y rotation angle here.
    double zRotation = 0;  // enter the desired Z rotation angle here.

    private PIDFController pidfController = new PIDFController(kP, kI, kD, kF);
    private MapleEncoder leftOdometerWheel, rightOdometerWheel, centerOdometerWheel;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftOdometerWheel = new MapleEncoder(
                hardwareMap.get(DcMotor.class, LEFT_ODOMETER_WHEEL_NAME),
                LEFT_ODOMETER_WHEEL_INVERTED,
                ODOMETER_ENCODER_TICKS_PER_REVOLUTION,
                1,
                ODOMETER_WHEELS_RADIUS_METERS
        );

        rightOdometerWheel = new MapleEncoder(
                hardwareMap.get(DcMotor.class, RIGHT_ODOMETER_WHEEL_NAME),
                RIGHT_ODOMETER_WHEEL_INVERTED,
                ODOMETER_ENCODER_TICKS_PER_REVOLUTION,
                1,
                ODOMETER_WHEELS_RADIUS_METERS
        );

        centerOdometerWheel = new MapleEncoder(
                hardwareMap.get(DcMotor.class, CENTER_ODOMETER_WHEEL_NAME),
                CENTER_ODOMETER_WHEEL_INVERTED,
                ODOMETER_ENCODER_TICKS_PER_REVOLUTION,
                1,
                ODOMETER_WHEELS_RADIUS_METERS
        );

        imu = hardwareMap.get(IMU.class, "imu");
        Orientation hubRotation = xyzOrientation(xRotation, yRotation, zRotation);

        // Now initialize the IMU with this mounting orientation
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(hubRotation);
        imu.initialize(new IMU.Parameters(orientationOnRobot));


        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        double speed = 0.7;
        double power1 = speed* (-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x);
        double power2 = speed* (-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x);


        waitForStart();
        while (opModeIsActive()) {
            pidfController.calculate();
            frontLeft.setPower(speed* power2);
            frontRight.setPower(-speed* power1);
            backLeft.setPower(speed* power2);
            backRight.setPower(speed* power1);
            telemetry.addData("current velocity",(leftOdometerWheel.getVelocityRevolutionsPerSecond()
                    +rightOdometerWheel.getVelocityRevolutionsPerSecond())/2);
            telemetry.addData("joystick y",gamepad1.left_stick_y);
            telemetry.addData("joystick lx",gamepad1.left_stick_x);
            telemetry.addData("joystick rx",gamepad1.right_stick_x);



            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

            telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.addData("Pitch (X)", "%.2f Deg.", orientation.getPitch(AngleUnit.DEGREES));
            telemetry.addData("Roll (Y)", "%.2f Deg.\n", orientation.getRoll(AngleUnit.DEGREES));
            telemetry.addData("Yaw (Z) velocity", "%.2f Deg/Sec", angularVelocity.zRotationRate);
            telemetry.addData("Pitch (X) velocity", "%.2f Deg/Sec", angularVelocity.xRotationRate);
            telemetry.addData("Roll (Y) velocity", "%.2f Deg/Sec", angularVelocity.yRotationRate);
            telemetry.update();
        }
}
}
