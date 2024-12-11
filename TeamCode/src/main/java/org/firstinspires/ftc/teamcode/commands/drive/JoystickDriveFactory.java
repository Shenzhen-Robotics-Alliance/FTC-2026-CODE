package org.firstinspires.ftc.teamcode.commands.drive;

import com.arcrobotics.ftclib.command.Command;

import org.firstinspires.ftc.teamcode.constants.DriveControlLoops;
import org.firstinspires.ftc.teamcode.constants.SystemConstants;
import org.firstinspires.ftc.teamcode.subsystems.drive.HolonomicDriveSubsystem;
import org.firstinspires.ftc.teamcode.utils.MapleJoystickDriveInput;

import java.util.concurrent.atomic.AtomicReference;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class JoystickDriveFactory {
    public static Command joystickDrive(HolonomicDriveSubsystem driveSubsystem, MapleJoystickDriveInput driveInput, BooleanSupplier driverStationCentricModeSwitch) {
        return driveSubsystem.drive(
                () -> {
                    final ChassisSpeeds desiredDriveStationCentricSpeed = driveInput.getJoystickChassisSpeeds(
                            driveSubsystem.getChassisMaxLinearVelocity(), driveSubsystem.getChassisMaxAngularVelocity() * 0.8
                    );
                    SystemConstants.telemetry.addData("Driver Station Requested Speed", desiredDriveStationCentricSpeed);
                    return desiredDriveStationCentricSpeed;
                },
                driverStationCentricModeSwitch
        );
    }

    public static Command joystickDrive(HolonomicDriveSubsystem driveSubsystem, MapleJoystickDriveInput driveInput, DoubleSupplier rotationXSupplier, DoubleSupplier rotationYSupplier) {
        final Runnable resetProfiledController = () -> DriveControlLoops.driveController
                .getThetaController()
                .reset(driveSubsystem.getFacing().getRadians());
        final AtomicReference<Rotation2d> desiredRotation = new AtomicReference<>(Rotation2d.kCCW_90deg);
        return driveSubsystem.drive(
                () -> {
                    final ChassisSpeeds driverInputSpeeds = driveInput.getJoystickChassisSpeeds(
                            driveSubsystem.getChassisMaxLinearVelocity(), 0);
                    final Translation2d rotationVector = new Translation2d(rotationXSupplier.getAsDouble(), rotationYSupplier.getAsDouble());
                    if (rotationVector.getNorm() > 0.5)
                        desiredRotation.set(rotationVector.getAngle());
                    final ChassisSpeeds feedBackSpeeds = DriveControlLoops.driveController.calculate(
                            driveSubsystem.getPoseWithVelocityCompensation(),
                            driveSubsystem.getPoseWithVelocityCompensation(),
                            0,
                            desiredRotation.get());
                    SystemConstants.telemetry.addData("CurrentRot (deg)", driveSubsystem.getPoseWithVelocityCompensation().getRotation().getDegrees());
                    SystemConstants.telemetry.addData("Desired Rot (deg)", desiredRotation.get().getDegrees());
                    SystemConstants.telemetry.addData("FeedBack AngularVelocity", Math.toDegrees(feedBackSpeeds.omegaRadiansPerSecond));
                    return new ChassisSpeeds(
                            driverInputSpeeds.vxMetersPerSecond,
                            driverInputSpeeds.vyMetersPerSecond,
                            feedBackSpeeds.omegaRadiansPerSecond);
                }, () -> true)
                .beforeStarting(resetProfiledController)
                .beforeStarting(() -> desiredRotation.set(driveSubsystem.getFacing()));
    }
}
