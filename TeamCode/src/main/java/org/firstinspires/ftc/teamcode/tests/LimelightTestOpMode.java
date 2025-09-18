package org.firstinspires.ftc.teamcode.tests;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.vision.LimelightSubsystem;

@TeleOp(name = "Limelight Telemetry Test")
public class LimelightTestOpMode extends CommandOpMode {


    private LimelightSubsystem limelight;
    private GamepadEx driverOp;

    @Override
    public void initialize() {

        limelight = new LimelightSubsystem(hardwareMap);
        driverOp = new GamepadEx(gamepad1);
        

        limelight.setPipeline(0);

        telemetry.setMsTransmissionInterval(11);
        telemetry.addData("Has Target", "Press A to Switch Pipeline");
        telemetry.addLine("---");

        telemetry.addData("Target Found", () -> limelight.hasTarget());
        telemetry.addData("Target ID", () -> limelight.getTargetID());
        telemetry.addData("Target X", () -> String.format("%.2f", limelight.getTargetX()));
        telemetry.addData("Target Y", () -> String.format("%.2f", limelight.getTargetY()));

    
    }
    @Override
    public void run() {
        telemetry.update(); 
    }
}