package org.firstinspires.ftc.teamcode.autos;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.RobotContainer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class AutoUtils {
    public static final Pose2d scoreShortBallsPose = new Pose2d(0.84, -0.4, Rotation2d.fromDegrees(0));
    public static final Pose2d scoreLongBallsPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    public static final Pose2d startPose = new Pose2d(0,0,Rotation2d.fromDegrees(-90));

    //Drive to short shooting pose and then shooting
    public static Command driveToShortPoseAndShot(RobotContainer robotContainer,Translation2d startingPoint,long ReturnTimeout) {
        final SequentialCommandGroup sequence = new SequentialCommandGroup();

        Command moveToShortScoringBalls = robotContainer.driveSubsystem.followPath(
                new Pose2d(startPose.getTranslation(),Rotation2d.fromDegrees(90)),
                new Translation2d[]{},  //change as the real situation
                new Pose2d(scoreShortBallsPose.getTranslation(),Rotation2d.fromDegrees(100)),
                Rotation2d.fromDegrees(45),
                0.7
        );

        sequence.addCommands(moveToShortScoringBalls.withTimeout(ReturnTimeout));

        sequence.addCommands(robotContainer.intakeCommand.intakeContinuously()
                .alongWith(robotContainer.shootCommand.fixShootShortContinuously())
                .withTimeout((long)3000));
        new WaitCommand(3500);
        sequence.addCommands(robotContainer.shootCommand.shootStop());



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
                () -> new ChassisSpeeds(-0.7, 0, 0),
                () -> false).withTimeout(1300)
                .alongWith(robotContainer.intakeCommand.intakeContinuously());
        sequence.addCommands(driveAndIntake);
        new WaitCommand(2000);

        return sequence;
    }

}