package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.constants.DriveTrainConstants.BACK_LEFT_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.constants.DriveTrainConstants.BACK_RIGHT_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.constants.DriveTrainConstants.CENTER_ODOMETER_WHEEL_INVERTED;
import static org.firstinspires.ftc.teamcode.constants.DriveTrainConstants.CENTER_ODOMETER_WHEEL_NAME;
import static org.firstinspires.ftc.teamcode.constants.DriveTrainConstants.FRONT_LEFT_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.constants.DriveTrainConstants.FRONT_RIGHT_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.constants.DriveTrainConstants.IMU_PARAMS;
import static org.firstinspires.ftc.teamcode.constants.DriveTrainConstants.LEFT_ODOMETER_WHEEL_INVERTED;
import static org.firstinspires.ftc.teamcode.constants.DriveTrainConstants.LEFT_ODOMETER_WHEEL_NAME;
import static org.firstinspires.ftc.teamcode.constants.DriveTrainConstants.ODOMETER_CENTER_WHEELS_OFFSET;
import static org.firstinspires.ftc.teamcode.constants.DriveTrainConstants.ODOMETER_ENCODER_TICKS_PER_REVOLUTION;
import static org.firstinspires.ftc.teamcode.constants.DriveTrainConstants.ODOMETER_WHEELS_RADIUS_METERS;
import static org.firstinspires.ftc.teamcode.constants.DriveTrainConstants.ODOMETER_WHEELS_TRACK_WIDTH_METERS;
import static org.firstinspires.ftc.teamcode.constants.DriveTrainConstants.RIGHT_ODOMETER_WHEEL_INVERTED;
import static org.firstinspires.ftc.teamcode.constants.DriveTrainConstants.RIGHT_ODOMETER_WHEEL_NAME;

import com.arcrobotics.ftclib.command.FunctionalCommand;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.constants.SystemConstants;
import org.firstinspires.ftc.teamcode.subsystems.drive.MapleOdometerWheelsOdometry;
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.utils.AllianceSide;
import org.firstinspires.ftc.teamcode.utils.MapleOdometerWheels.MapleEncoder;

import java.io.Closeable;
import java.io.IOException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * declares all the subsystems of a robot
 * */
public final class RobotContainer implements Closeable {
    public final AllianceSide currentSide;

    public final MecanumDriveSubsystem driveSubsystem;

    public final MapleOdometerWheelsOdometry testOdometry;

    // public final AprilTagVision vision;
    /** create all the subsystem with the hardware map */
    public RobotContainer(HardwareMap hardwareMap, AllianceSide side) {
        this.currentSide = side;

        /* here we creates all the subsystems */
        this.testOdometry = new MapleOdometerWheelsOdometry(hardwareMap, new Pose2d());
        testOdometry.register();
        testOdometry.setDefaultCommand(new FunctionalCommand(
                () -> {},
                () -> SystemConstants.telemetry.addData("Estimated Pose", testOdometry.getEstimatedPose()),
                (Boolean terminated) -> {},
                () -> false,
                testOdometry
        ));

        this.driveSubsystem = new MecanumDriveSubsystem(hardwareMap, testOdometry);
    }

    @Override
    public void close() throws IOException {
        // vision.close();
    }
}
