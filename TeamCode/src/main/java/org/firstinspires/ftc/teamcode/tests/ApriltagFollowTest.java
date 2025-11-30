package org.firstinspires.ftc.teamcode.tests;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.subsystems.vision.VisionSubsystem;
@TeleOp(name = "Apriltag Follow Test")
public class ApriltagFollowTest extends OpMode {
    private DcMotor RotMotor;
    @Override
    public void init() {
        RotMotor = hardwareMap.get(DcMotor.class, "RotMotor");

    }

    @Override
    public void loop() {
        VisionSubsystem limelight = new VisionSubsystem(hardwareMap);

        limelight.setPipeline(0);

        telemetry.setMsTransmissionInterval(11);
        telemetry.addData("Has Target", "Press A to Switch Pipeline");
        telemetry.addLine("---");

        telemetry.addData("Target Found", () -> limelight.hasTarget());
        telemetry.addData("Target ID", () -> limelight.getCurrentID());
        telemetry.addData("Target X", () -> String.format("%.2f", limelight.targetX));
        telemetry.addData("Target Y", () -> String.format("%.2f", limelight.targetY));
        RotMotor.setPower(limelight.targetX*-0.3);

    }
}
