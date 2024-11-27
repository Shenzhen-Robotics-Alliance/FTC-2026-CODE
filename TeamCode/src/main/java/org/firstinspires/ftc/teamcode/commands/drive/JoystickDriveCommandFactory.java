package org.firstinspires.ftc.teamcode.commands.drive;

import com.arcrobotics.ftclib.command.Command;

import org.firstinspires.ftc.teamcode.constants.SystemConstants;
import org.firstinspires.ftc.teamcode.subsystems.drive.HolonomicDriveSubsystem;
import org.firstinspires.ftc.teamcode.utils.MapleJoystickDriveInput;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class JoystickDriveCommandFactory {
    public static Command joystickDrive(HolonomicDriveSubsystem driveSubsystem, MapleJoystickDriveInput driveInput, BooleanSupplier driverStationCentricModeSwitch) {
        return driveSubsystem.driveCommand(
                () -> {
                    final ChassisSpeeds desiredDriveStationCentricSpeed = driveInput.getJoystickChassisSpeeds(
                            driveSubsystem.getChassisMaxLinearVelocity(), driveSubsystem.getChassisMaxAngularVelocity()
                    );
                    SystemConstants.telemetry.addData("Driver Station Requested Speed", desiredDriveStationCentricSpeed);
                    return desiredDriveStationCentricSpeed;
                },
                driverStationCentricModeSwitch
        );
    }
}
