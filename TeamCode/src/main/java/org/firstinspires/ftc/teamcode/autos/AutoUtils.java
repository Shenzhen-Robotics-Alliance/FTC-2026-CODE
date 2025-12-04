package org.firstinspires.ftc.teamcode.autos;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.RobotContainer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class AutoUtils {
    public static final Pose2d scoreShortBallsPose = new Pose2d(-0.01, 0.39, Rotation2d.fromDegrees(15));
    public static final Pose2d scoreLongBallsPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    public static final Pose2d startPose = new Pose2d(0,0,Rotation2d.fromDegrees(0));

    //Drive to short shooting pose and then shooting
    public static Command driveToShortPoseAndShot(RobotContainer robotContainer) {
        final SequentialCommandGroup sequence = new SequentialCommandGroup();

        Command moveToShortScoringBalls = robotContainer.driveSubsystem.followPath(
                new Pose2d(startPose.getTranslation(),Rotation2d.fromDegrees(0)),
                new Translation2d[]{new Translation2d(0.15,0.03)},  //change as the real situation
                new Pose2d(Positions.SHOOTING_POINT,Rotation2d.fromDegrees(15)),
                Rotation2d.fromDegrees(15),
                0.3
        );

        sequence.addCommands(moveToShortScoringBalls);

        sequence.addCommands(robotContainer.intakeCommand.intakeContinuously());

        sequence.addCommands(robotContainer.shootCommand.fixShootShortContinuously().withTimeout((long)3000));

        sequence.addCommands(robotContainer.intakeCommand.stopIntake());


        return sequence;
    }

    //Drive to long shooting pose and then shooting
    public static Command driveToFarPoseAndShot(RobotContainer robotContainer) {
        final SequentialCommandGroup sequence = new SequentialCommandGroup();

        Command moveToFarScoringBalls = robotContainer.driveSubsystem.followPath(
                new Pose2d(startPose.getTranslation(),Rotation2d.fromDegrees(0)),
                new Translation2d[]{new Translation2d(0,0)},   //change as the real situation
                new Pose2d(Positions.SHOOTING_POINT,Rotation2d.fromDegrees(0)),
                Rotation2d.fromDegrees(0),
                0.3
        );

        sequence.addCommands(moveToFarScoringBalls);

        sequence.addCommands(robotContainer.shootCommand.fixShootFarContinuously());

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