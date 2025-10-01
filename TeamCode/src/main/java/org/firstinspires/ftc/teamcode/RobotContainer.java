package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.FunctionalCommand;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.commands.intakeCommands.IntakeContinueCommand;
import org.firstinspires.ftc.teamcode.commands.intakeCommands.IntakeStop;
import org.firstinspires.ftc.teamcode.commands.intakeCommands.OuttakeContinueCommand;
import org.firstinspires.ftc.teamcode.commands.shotCommands.RotCommands;
import org.firstinspires.ftc.teamcode.constants.SystemConstants;
import org.firstinspires.ftc.teamcode.subsystems.SuperStructure.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SuperStructure.RotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.drive.MapleOdometerWheelsOdometry;
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.utils.AllianceSide;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * declares all the subsystems of a robot
 * */
public final class RobotContainer implements AutoCloseable {
    public final AllianceSide currentSide;

    public final MecanumDriveSubsystem driveSubsystem;
    public final IntakeSubsystem intakeSubsystem;
    public final IntakeContinueCommand intakeContinueCommand;
    public final IntakeStop intakeStop;
    public final OuttakeContinueCommand outtakeContinueCommand;
    public final RotSubsystem rotSubsystem;
    public final RotCommands rotCommands;

    public final MapleOdometerWheelsOdometry odometry;

    // public final AprilTagVision vision;
    /** create all the subsystem with the hardware map */
    public RobotContainer(HardwareMap hardwareMap, AllianceSide side) {
        this.currentSide = side;

        /* here we creates all the subsystems */
        this.odometry = new MapleOdometerWheelsOdometry(hardwareMap, new Pose2d());
        odometry.register();
        odometry.setDefaultCommand(new FunctionalCommand(
                () -> {},
                () -> SystemConstants.telemetry.addData("Estimated Pose", odometry.getEstimatedPose()),
                (Boolean terminated) -> {},
                () -> false,
                odometry
        ));

        this.driveSubsystem = new MecanumDriveSubsystem(hardwareMap, odometry);

        this.intakeSubsystem = new IntakeSubsystem(hardwareMap);
        this.intakeContinueCommand = new IntakeContinueCommand(intakeSubsystem);
        this.intakeStop = new IntakeStop(intakeSubsystem);
        this.outtakeContinueCommand = new OuttakeContinueCommand(intakeSubsystem);

        this.rotSubsystem = new RotSubsystem(hardwareMap);
        this.rotCommands = new RotCommands(rotSubsystem);
    }

    @Override
    public void close() throws Exception {
        odometry.close();
    }
}
