package org.firstinspires.ftc.teamcode.subsystems.drive;

import static org.firstinspires.ftc.teamcode.constants.DriveTrainConstants.*;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.constants.SystemConstants;
import org.firstinspires.ftc.teamcode.utils.MapleOdometerWheels.MapleEncoder;
import org.firstinspires.ftc.teamcode.utils.MapleOdometerWheels.OdometerWheelsPoseEstimator;
import org.firstinspires.ftc.teamcode.utils.MapleOdometerWheels.OdometerWheelsPositions;
import org.firstinspires.ftc.teamcode.utils.MapleOdometerWheels.OdometerWheelsSpeeds;
import org.firstinspires.ftc.teamcode.utils.MapleTimer;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class MapleOdometerWheelsOdometry implements Subsystem {
    private final OdometerWheelsPoseEstimator poseEstimator;
    private final MapleEncoder leftOdometerWheel, rightOdometerWheel, centerOdometerWheel;
    private final IMU imu;
    private Rotation2d currentRotation;
    private double previousIMUUpdateTimeSeconds = 0;
    private OdometerWheelsPositions previousPositions;

    public MapleOdometerWheelsOdometry(HardwareMap hardwareMap, Pose2d initialPose) {
        this.leftOdometerWheel = new MapleEncoder(
                hardwareMap.get(DcMotor.class, LEFT_ODOMETER_WHEEL_NAME),
                LEFT_ODOMETER_WHEEL_INVERTED,
                ODOMETER_ENCODER_TICKS_PER_REVOLUTION,
                1,
                ODOMETER_WHEELS_RADIUS_METERS,
                50
        );
        this.rightOdometerWheel = new MapleEncoder(
                hardwareMap.get(DcMotor.class, RIGHT_ODOMETER_WHEEL_NAME),
                RIGHT_ODOMETER_WHEEL_INVERTED,
                ODOMETER_ENCODER_TICKS_PER_REVOLUTION,
                1,
                ODOMETER_WHEELS_RADIUS_METERS,
                50
        );
        this.centerOdometerWheel = new MapleEncoder(
                hardwareMap.get(DcMotor.class, CENTER_ODOMETER_WHEEL_NAME),
                CENTER_ODOMETER_WHEEL_INVERTED,
                ODOMETER_ENCODER_TICKS_PER_REVOLUTION,
                1,
                ODOMETER_WHEELS_RADIUS_METERS,
                50
        );

        this.imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(IMU_PARAMS);

        pollEncodersBlocking();
        poseEstimator = new OdometerWheelsPoseEstimator(
                ODOMETER_WHEELS_TRACK_WIDTH_METERS, ODOMETER_CENTER_WHEELS_OFFSET,
                getIMUAngleBlocking(),
                getLatestPositions(),
                initialPose
        );

        this.currentRotation = initialPose.getRotation();
        this.previousPositions = getLatestPositions();

        resetPose(initialPose);
    }

    private void pollEncodersBlocking() {
        leftOdometerWheel.poll();
        rightOdometerWheel.poll();
        centerOdometerWheel.poll();
    }

    /**
    * poll the newest reading from imu
    * note that this method will block the thread for 10~20 ms (because the control hub sucks)
    * so avoid calling it periodically
    * */
    private Rotation2d getIMUAngleBlocking() {
        return Rotation2d.fromRadians(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
    }

    public OdometerWheelsPositions getLatestPositions() {
        return new OdometerWheelsPositions(
                leftOdometerWheel.getDistanceMeters(),
                rightOdometerWheel.getDistanceMeters(),
                centerOdometerWheel.getDistanceMeters()
        );
    }

    public OdometerWheelsSpeeds getLatestSpeeds() {
        return new OdometerWheelsSpeeds(
                leftOdometerWheel.getVelocityMetersPerSecond(),
                rightOdometerWheel.getVelocityMetersPerSecond(),
                centerOdometerWheel.getVelocityMetersPerSecond()
        );
    }

    /**
     * fetches the data cached by encoder thread (if encoder thread enabled) and feed these data to the pose estimator
     * */
    @Override
    public void periodic() {
        pollEncodersBlocking();
        final Twist2d twist2d = poseEstimator.kinematics.toTwist2d(previousPositions, getLatestPositions());
        previousPositions = getLatestPositions();

        if (MapleTimer.getMatchTimeSeconds() - previousIMUUpdateTimeSeconds > 1.0/ SystemConstants.IMU_UPDATE_HZ) {
            currentRotation = getIMUAngleBlocking();
            previousIMUUpdateTimeSeconds = MapleTimer.getMatchTimeSeconds();
        } else
            currentRotation = currentRotation.plus(Rotation2d.fromRadians(twist2d.dtheta));

        poseEstimator.updateWithTime(
                MapleTimer.getMatchTimeSeconds(),
                currentRotation,
                getLatestPositions()
        );

        SystemConstants.telemetry.addData("Measured Speeds", getMeasureSpeedsRobotRelative());
    }

    public void resetPose(Pose2d currentPose) {
        pollEncodersBlocking();
        this.poseEstimator.resetPosition(getIMUAngleBlocking(), getLatestPositions(), currentPose);
    }

    public void addVisionMeasurement(Pose2d visionRobotPose, double timeStamp) {
        this.poseEstimator.addVisionMeasurement(visionRobotPose, timeStamp);
    }

    public void addVisionMeasurement(Pose2d visionRobotPose, double timeStamp, Matrix<N3, N1> visionMeasurementStdDev) {
        this.poseEstimator.addVisionMeasurement(visionRobotPose, timeStamp, visionMeasurementStdDev);
    }

    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public ChassisSpeeds getMeasureSpeedsRobotRelative() {
        return this.poseEstimator.kinematics.toChassisSpeeds(getLatestSpeeds());
    }
}
