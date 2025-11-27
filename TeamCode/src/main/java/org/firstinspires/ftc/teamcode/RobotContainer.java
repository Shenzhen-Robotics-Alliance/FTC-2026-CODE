package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.FunctionalCommand;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.commands.intakeCommands.IntakeCommand;
//import org.firstinspires.ftc.teamcode.commands.intakeCommands.IntakeContinueCommand;
//import org.firstinspires.ftc.teamcode.commands.intakeCommands.IntakeStop;
//import org.firstinspires.ftc.teamcode.commands.intakeCommands.OuttakeContinueCommand;
import org.firstinspires.ftc.teamcode.commands.shotCommands.AutoRotCommand;
import org.firstinspires.ftc.teamcode.commands.shotCommands.ManualRotCommand;
import org.firstinspires.ftc.teamcode.commands.shotCommands.ShootCommand;
import org.firstinspires.ftc.teamcode.commands.visionCommands.FollowCommand;
import org.firstinspires.ftc.teamcode.constants.SystemConstants;
import org.firstinspires.ftc.teamcode.subsystems.SuperStructure.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SuperStructure.PreShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SuperStructure.RotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SuperStructure.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.drive.MapleOdometerWheelsOdometry;
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.vision.VisionSubsystem;
import org.firstinspires.ftc.teamcode.utils.AllianceSide;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * declares all the subsystems of a robot
 * */
public final class RobotContainer implements AutoCloseable {
    public final AllianceSide currentSide;

    public final MecanumDriveSubsystem driveSubsystem;
    public final IntakeSubsystem intakeSubsystem;
    public final PreShooterSubsystem preShooterSubsystem;
    public final ShooterSubsystem shooterSubsystem ;

    public final IntakeCommand intakeCommand;
    public final RotSubsystem rotSubsystem;
    public final ManualRotCommand manualRotCommand;
    public final AutoRotCommand autoRotCommand;

    public final MapleOdometerWheelsOdometry odometry;


    private Supplier<Double> joystickSupplier;

    public final ShootCommand shootCommand;

    public final VisionSubsystem visionSubsystem;

    public final FollowCommand followCommand;


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

        this.preShooterSubsystem = new PreShooterSubsystem(hardwareMap);

        this.visionSubsystem = new VisionSubsystem(hardwareMap);
        this.followCommand = new FollowCommand(visionSubsystem);

        this.rotSubsystem = new RotSubsystem(hardwareMap);
        rotSubsystem.register();
        this.manualRotCommand = new ManualRotCommand(rotSubsystem,joystickSupplier);
        this.autoRotCommand = new AutoRotCommand(rotSubsystem,visionSubsystem,side);

        this.shooterSubsystem = new ShooterSubsystem(hardwareMap);
        shooterSubsystem.register();
        this.shootCommand = new ShootCommand(shooterSubsystem,preShooterSubsystem,driveSubsystem);


        this.intakeSubsystem = new IntakeSubsystem(hardwareMap);
        this.intakeCommand = new IntakeCommand(intakeSubsystem);


    }

    @Override
    public void close() throws Exception {
        odometry.close();
    }
}
