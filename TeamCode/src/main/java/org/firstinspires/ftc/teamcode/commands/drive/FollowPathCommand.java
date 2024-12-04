package org.firstinspires.ftc.teamcode.commands.drive;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.drive.HolonomicDriveSubsystem;
import org.firstinspires.ftc.teamcode.utils.MapleTimer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;

import static org.firstinspires.ftc.teamcode.constants.DriveControlLoops.*;

public class FollowPathCommand extends CommandBase {
    private final Trajectory trajectory;
    private final Rotation2d desiredRotation;
    private final double delaySecondsStartRotating;
    private final Pose2d tolerance;
    private final HolonomicDriveSubsystem driveSubsystem;

    private final MapleTimer timer;

    public FollowPathCommand(Trajectory trajectory, HolonomicDriveSubsystem driveSubsystem, Rotation2d desiredRotation) {
        this(trajectory, driveSubsystem, desiredRotation, 0);
    }

    public FollowPathCommand(Trajectory trajectory, HolonomicDriveSubsystem driveSubsystem, Rotation2d desiredRotation, double pathProgressPercentageStartRotating) {
        this(trajectory, driveSubsystem, desiredRotation, pathProgressPercentageStartRotating, new Pose2d(0.05, 0.05, Rotation2d.fromDegrees(5)));
    }
    public FollowPathCommand(Trajectory trajectory, HolonomicDriveSubsystem driveSubsystem, Rotation2d desiredRotation, double pathProgressPercentageStartRotating, Pose2d tolerance) {
        this.trajectory = trajectory;
        this.desiredRotation = desiredRotation;

        this.delaySecondsStartRotating = pathProgressPercentageStartRotating * trajectory.getTotalTimeSeconds();
        this.driveSubsystem = driveSubsystem;

        this.tolerance = tolerance;
        this.timer = new MapleTimer();

        super.addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        driveController.setTolerance(tolerance);
        timer.reset();
        driveController.getThetaController().reset(driveSubsystem.getFacing().getRadians());
    }

    @Override
    public void execute() {
        final Trajectory.State state = trajectory.sample(timer.getTimeSeconds());
        driveSubsystem.runRobotCentricChassisSpeeds(driveController.calculate(
                driveSubsystem.getPose(),
                state,
                timer.getTimeSeconds() > delaySecondsStartRotating ?
                        desiredRotation
                        : driveSubsystem.getFacing()
                ));
    }

    @Override
    public boolean isFinished() {
        return timer.getTimeSeconds() >= trajectory.getTotalTimeSeconds()
                && driveController.atReference();
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stop();
    }

    public FollowPathCommand withTimeOutAfterTrajectoryFinished(double timeOutSeconds) {
        return (FollowPathCommand) this.withTimeout((long) (
                (trajectory.getTotalTimeSeconds() + timeOutSeconds) * 1000
        ));
    }

    public Command withNavigateToStartingPoint(Pose2d startingPointTolerance, double navigateTimeOutSeconds) {
        return driveSubsystem
                .driveToPose(
                        () -> new Pose2d(trajectory.getInitialPose().getTranslation(), driveSubsystem.getFacing()),
                        startingPointTolerance,
                        navigateTimeOutSeconds)
                .andThen(this);
    }
}
