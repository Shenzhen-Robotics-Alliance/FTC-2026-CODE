package org.firstinspires.ftc.teamcode.autos;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.RobotContainer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class AutoUtils {
    public static final Pose2d scoreShortBallsPose = new Pose2d(0, 0, Rotation2d.fromDegrees(45));
    public static final Pose2d scoreLongBallsPose = new Pose2d(0, 0, Rotation2d.fromDegrees(45));


    //Drive to short shooting pose and then shooting
    public static Command driveToShortPoseAndShot(RobotContainer robotContainer) {
        final SequentialCommandGroup sequence = new SequentialCommandGroup();
        Command moveToShortScoringBalls = robotContainer.driveSubsystem.driveToPose(
                () -> scoreShortBallsPose,
                new Pose2d(0.02,0.02,Rotation2d.fromDegrees(5)),
                1
        );
        sequence.addCommands(moveToShortScoringBalls);

        sequence.addCommands(robotContainer.shootCommand.shootShortContinuously());

        return sequence;
    }

    //Drive to long shooting pose and then shooting
    public static Command driveToFarPoseAndShot(RobotContainer robotContainer) {
        final SequentialCommandGroup sequence = new SequentialCommandGroup();
        Command moveToFarScoringBalls = robotContainer.driveSubsystem.driveToPose(
                () -> scoreLongBallsPose,
                new Pose2d(0,0,Rotation2d.fromDegrees(5)),
                1
        );
        sequence.addCommands(moveToFarScoringBalls);

        sequence.addCommands(robotContainer.shootCommand.shootFarContinuously());

        return sequence;
    }

    public static Command driveToIntakeContinuousLy(RobotContainer robotContainer){
        final SequentialCommandGroup sequence = new SequentialCommandGroup();
        Command driveAndIntake = robotContainer.driveSubsystem.drive(
                () -> new ChassisSpeeds(0.3, 0, 0),
                () -> false).withTimeout(1200)
                .alongWith(robotContainer.intakeCommand.intakeContinuously());
        sequence.addCommands(driveAndIntake);

        return sequence;
    }

}