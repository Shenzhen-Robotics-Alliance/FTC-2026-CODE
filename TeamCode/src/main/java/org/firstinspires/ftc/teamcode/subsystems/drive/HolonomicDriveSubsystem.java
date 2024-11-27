package org.firstinspires.ftc.teamcode.subsystems.drive;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.teamcode.constants.SystemConstants;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public interface HolonomicDriveSubsystem extends Subsystem {
    /**
     * runs a ChassisSpeeds without doing any pre-processing
     * @param speeds a discrete chassis speed, robot-centric
     * */
    void runRawChassisSpeeds(ChassisSpeeds speeds);

    /**
     * Returns the current odometry Pose.
     */
    Pose2d getPose();

    default Rotation2d getFacing() {return getPose().getRotation(); }

    /**
     * Resets the current odometry Pose to a given Pose
     */
    void setPose(Pose2d currentPose);

    /**
     * @return the measured(actual) velocities of the chassis, robot-relative
     * */
    ChassisSpeeds getMeasuredChassisSpeedsRobotRelative();

    default ChassisSpeeds getMeasuredChassisSpeedsFieldRelative() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(getMeasuredChassisSpeedsRobotRelative(), getFacing());
    }

    double getChassisMaxLinearVelocity();
    double getChassisMaxAngularVelocity();

    /**
     * Adds a vision measurement to the pose estimator.
     *
     * @param visionPose The pose of the robot as measured by the vision camera.
     * @param timestamp  The timestamp of the vision measurement in seconds.
     */
    void addVisionMeasurement(Pose2d visionPose, double timestamp);

    /**
     * runs a field-centric ChassisSpeeds
     * @param fieldCentricSpeeds a continuous chassis speeds, field-centric, normally from a pid position controller
     * */
    default void runFieldCentricChassisSpeeds(ChassisSpeeds fieldCentricSpeeds) {
        runRobotCentricChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
                fieldCentricSpeeds,
                getPose().getRotation()
        ));
    }

    default void stop() {
        runRobotCentricChassisSpeeds(new ChassisSpeeds());
    }

    /**
     * runs a ChassisSpeeds, pre-processed with ChassisSpeeds.discretize()
     * @param speeds a continuous chassis speed, robot-centric
     * */
    default void runRobotCentricChassisSpeeds(ChassisSpeeds speeds) {
        runRawChassisSpeeds(ChassisSpeeds.discretize(
                speeds,
                1.0/ SystemConstants.ROBOT_UPDATE_RATE_HZ
        ));
    }

    static boolean isZero(ChassisSpeeds chassisSpeeds) {
        return chassisSpeeds.omegaRadiansPerSecond == 0 && chassisSpeeds.vxMetersPerSecond == 0 && chassisSpeeds.vyMetersPerSecond == 0;
    }

    default Command driveCommand(Supplier<ChassisSpeeds> chassisSpeedsSupplier, BooleanSupplier fieldCentricSwitch) {
        return new RunCommand(
                () -> {
                    if (fieldCentricSwitch.getAsBoolean())
                        runFieldCentricChassisSpeeds(chassisSpeedsSupplier.get());
                   else
                       runRobotCentricChassisSpeeds(chassisSpeedsSupplier.get());
                },
                this
        );
    }
}
