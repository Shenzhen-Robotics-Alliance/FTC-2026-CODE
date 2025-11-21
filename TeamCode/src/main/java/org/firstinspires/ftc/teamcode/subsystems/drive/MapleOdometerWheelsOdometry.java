package org.firstinspires.ftc.teamcode.subsystems.drive;

import static org.firstinspires.ftc.teamcode.constants.DriveTrainConstants.*;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.constants.SystemConstants;
import org.firstinspires.ftc.teamcode.utils.MapleLoopClock;
import org.firstinspires.ftc.teamcode.utils.MapleOdometerWheels.MapleEncoder;
import org.firstinspires.ftc.teamcode.utils.MapleOdometerWheels.OdometerWheelsPoseEstimator;
import org.firstinspires.ftc.teamcode.utils.MapleOdometerWheels.OdometerWheelsPositions;
import org.firstinspires.ftc.teamcode.utils.MapleOdometerWheels.OdometerWheelsSpeeds;
import org.firstinspires.ftc.teamcode.utils.MapleTimer;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
“”“”“”“”“”“”“”“”“”“”“”“”“”“”“”“”“”“”“
public class MapleOdometerWheelsOdometry implements Subsystem, AutoCloseable {
    private boolean running = true;

    private final Thread odometryThread, imuThread;
    private final Lock poseEstimatorLock = new ReentrantLock();

    private final OdometerWheelsPoseEstimator poseEstimator;
    private final MapleEncoder leftOdometerWheel, rightOdometerWheel, centerOdometerWheel;
    private final IMU imu;
    private Rotation2d currentRotation;
    private Rotation2d imuOffset;
    private OdometerWheelsPositions previousPositions;

    private int odometryTicksInRobotPeriod = 0;

    public MapleOdometerWheelsOdometry(HardwareMap hardwareMap, Pose2d initialPose) {
        this.leftOdometerWheel = new MapleEncoder(
                hardwareMap.get(DcMotor.class, LEFT_ODOMETER_WHEEL_NAME),
                LEFT_ODOMETER_WHEEL_INVERTED,
                ODOMETER_ENCODER_TICKS_PER_REVOLUTION,
                1,
                ODOMETER_WHEELS_RADIUS_METERS,
                SystemConstants.ODOMETRY_UPDATE_RATE_HZ);
        this.rightOdometerWheel = new MapleEncoder(
                hardwareMap.get(DcMotor.class, RIGHT_ODOMETER_WHEEL_NAME),
                RIGHT_ODOMETER_WHEEL_INVERTED,
                ODOMETER_ENCODER_TICKS_PER_REVOLUTION,
                1,
                ODOMETER_WHEELS_RADIUS_METERS,
                SystemConstants.ODOMETRY_UPDATE_RATE_HZ);
        this.centerOdometerWheel = new MapleEncoder(
                hardwareMap.get(DcMotor.class, CENTER_ODOMETER_WHEEL_NAME),
                CENTER_ODOMETER_WHEEL_INVERTED,
                ODOMETER_ENCODER_TICKS_PER_REVOLUTION,
                1,
                ODOMETER_WHEELS_RADIUS_METERS,
                SystemConstants.ODOMETRY_UPDATE_RATE_HZ);

        this.imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(IMU_PARAMS);
        imu.resetYaw();

        poseEstimator = new OdometerWheelsPoseEstimator(
                ODOMETER_WHEELS_TRACK_WIDTH_METERS, ODOMETER_CENTER_WHEELS_OFFSET,
                getIMUAngleBlocking(),
                getLatestPositions(),
                initialPose);

        this.currentRotation = initialPose.getRotation();
        this.previousPositions = getLatestPositions();

        resetPose(initialPose);
        measuredSpeeds = new ChassisSpeeds();

        this.odometryThread = new Thread(() -> {
            final MapleLoopClock clock = new MapleLoopClock(SystemConstants.ODOMETRY_UPDATE_RATE_HZ);
            while (running) {
                clock.tick();
                updateEncoders();
                odometryTicksInRobotPeriod++;
            }
        });
        this.imuThread = new Thread(() -> {
            final MapleLoopClock clock = new MapleLoopClock(SystemConstants.IMU_UPDATE_HZ);
            while (running) {
                clock.tick();
                updateIMU();
            }
        });
        odometryThread.setDaemon(true);
        imuThread.setDaemon(true);
        odometryThread.start();
        imuThread.start();
    }

    private final Lock encodersLock = new ReentrantLock();
    private void pollEncodersBlocking() {
        encodersLock.lock();
        leftOdometerWheel.poll();
        rightOdometerWheel.poll();
        centerOdometerWheel.poll();
        encodersLock.unlock();
    }

    /**
    * poll the newest reading from imu
    * note that this method will block the thread for 10~20 ms (because the control hub sucks)
    * so avoid calling it periodically
    * */
    private final Lock imuLock = new ReentrantLock();
    private Rotation2d getIMUAngleBlocking() {
        imuLock.lock();
        Rotation2d imuAngle =  Rotation2d.fromRadians(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        imuLock.unlock();

        return imuAngle;
    }

    public OdometerWheelsPositions getLatestPositions() {
        return new OdometerWheelsPositions(
                leftOdometerWheel.getDistanceMeters(),
                rightOdometerWheel.getDistanceMeters(),
                centerOdometerWheel.getDistanceMeters());
    }

    public OdometerWheelsSpeeds getLatestSpeeds() {
        return new OdometerWheelsSpeeds(
                leftOdometerWheel.getVelocityMetersPerSecond(),
                rightOdometerWheel.getVelocityMetersPerSecond(),
                centerOdometerWheel.getVelocityMetersPerSecond());
    }

    @Override
    public void periodic() {
        poseEstimatorLock.lock();

        this.measuredSpeeds = poseEstimator.kinematics.toChassisSpeeds(getLatestSpeeds());
        this.estimatedPose = poseEstimator.getEstimatedPosition();
        SystemConstants.telemetry.addData("Measured Speeds", getMeasureSpeedsRobotRelative());
        SystemConstants.telemetry.addData("Odometry Ticks Per Period", odometryTicksInRobotPeriod);
        odometryTicksInRobotPeriod = 0;

        poseEstimatorLock.unlock();
    }

    /**
     * fetches the data cached by encoder thread (if encoder thread enabled) and feed these data to the pose estimator
     * */
    public void updateEncoders() {
        pollEncodersBlocking();

        final Twist2d twist2d = poseEstimator.kinematics.toTwist2d(previousPositions, getLatestPositions());
        previousPositions = getLatestPositions();
        currentRotation = currentRotation.plus(Rotation2d.fromRadians(twist2d.dtheta));

        poseEstimatorLock.lock();
        poseEstimator.updateWithTime(
                MapleTimer.getMatchTimeSeconds(),
                currentRotation,
                getLatestPositions());
        poseEstimatorLock.unlock();
    }

    public void updateIMU() {
        double time = MapleTimer.getMatchTimeSeconds();
        Rotation2d imuMeasurement = getIMUAngleBlocking();

        addVisionMeasurement(
                new Pose2d(estimatedPose.getTranslation(), imuMeasurement.plus(imuOffset)),
                time,
                VecBuilder.fill(9999, 9999, 0.002));
    }

    public void resetPose(Pose2d currentPose) {
        pollEncodersBlocking();
        Rotation2d imuAngle = getIMUAngleBlocking();

        this.poseEstimatorLock.lock();
        currentRotation = currentPose.getRotation();
        this.imuOffset = currentRotation.minus(imuAngle);
        estimatedPose = currentPose;
        this.poseEstimator.resetPosition(currentRotation, getLatestPositions(), currentPose);
        this.poseEstimatorLock.unlock();
    }

    public void addVisionMeasurement(Pose2d visionRobotPose, double timeStamp) {
        this.poseEstimatorLock.lock();
        this.poseEstimator.addVisionMeasurement(visionRobotPose, timeStamp);
        this.poseEstimatorLock.unlock();
    }

    public void addVisionMeasurement(Pose2d visionRobotPose, double timeStamp, Matrix<N3, N1> visionMeasurementStdDev) {
        this.poseEstimatorLock.lock();
        this.poseEstimator.addVisionMeasurement(visionRobotPose, timeStamp, visionMeasurementStdDev);
        this.poseEstimatorLock.unlock();
    }

    private Pose2d estimatedPose;
    public Pose2d getEstimatedPose() {
        return estimatedPose;
    }

    private ChassisSpeeds measuredSpeeds;
    public ChassisSpeeds getMeasureSpeedsRobotRelative() {
        return measuredSpeeds;
    }

    @Override
    public void close() throws Exception {
        running = false;
    }
}
