package org.firstinspires.ftc.teamcode.commands.drive;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.drive.HolonomicDriveSubsystem;
import org.firstinspires.ftc.teamcode.utils.MapleTime;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import static org.firstinspires.ftc.teamcode.constants.DriveTrainConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;


public class FollowPathCommand extends CommandBase {
    private final Trajectory trajectory;
    private final Rotation2d desiredRotation;
    private final double delaySecondsStartRotating;
    private final HolonomicDriveSubsystem driveSubsystem;
    private final HolonomicDriveController driveController;

    private double startTimeSeconds = 0;

    public FollowPathCommand(Trajectory trajectory, HolonomicDriveSubsystem driveSubsystem, Rotation2d desiredRotation) {
        this(trajectory, driveSubsystem, desiredRotation, 0);
    }

    public FollowPathCommand(Trajectory trajectory, HolonomicDriveSubsystem driveSubsystem, Rotation2d desiredRotation, double pathProgressPercentageStartRotating) {
        this(trajectory, driveSubsystem, desiredRotation, pathProgressPercentageStartRotating, new Pose2d(0.05, 0.05, Rotation2d.fromDegrees(5)));
    }
    public FollowPathCommand(Trajectory trajectory, HolonomicDriveSubsystem driveSubsystem, Rotation2d desiredRotation, double pathProgressPercentageStartRotating, Pose2d tolerance) {
        super.addRequirements(driveSubsystem);
        this.trajectory = trajectory;
        this.desiredRotation = desiredRotation;

        this.delaySecondsStartRotating = pathProgressPercentageStartRotating * trajectory.getTotalTimeSeconds();
        this.driveSubsystem = driveSubsystem;
        driveController = new HolonomicDriveController(
                new PIDController(5.0, 0, 0),
                new PIDController(5.0, 0 ,0),
                new ProfiledPIDController(5.0, 0, 0,
                        new TrapezoidProfile.Constraints(MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, Math.toRadians(720))));

        driveController.setTolerance(tolerance);
    }

    @Override
    public void initialize() {
        startTimeSeconds = MapleTime.getMatchTimeSeconds();
        driveController.getThetaController().reset(driveSubsystem.getFacing().getRadians());
    }

    @Override
    public void execute() {
        final Trajectory.State state = trajectory.sample(getPathTime());
        driveSubsystem.runRobotCentricChassisSpeeds(driveController.calculate(
                driveSubsystem.getPose(),
                state,
                getPathTime() > delaySecondsStartRotating ?
                        desiredRotation
                        : driveSubsystem.getFacing()
                ));
    }

    @Override
    public boolean isFinished() {
        return getPathTime() >= trajectory.getTotalTimeSeconds()
                && driveController.atReference();
    }

    public double getPathTime() {
        return MapleTime.getMatchTimeSeconds() - startTimeSeconds;
    }
}
