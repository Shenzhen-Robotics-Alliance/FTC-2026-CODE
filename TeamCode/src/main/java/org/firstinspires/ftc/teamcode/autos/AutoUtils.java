//=============AutoUtils==========
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
    public static final Pose2d scoreShortBallsPose = new Pose2d(0.78, -0.49, Rotation2d.fromDegrees(0));
    public static final Pose2d scoreLongBallsPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    public static final Pose2d startPose = new Pose2d(0,0,Rotation2d.fromDegrees(0));

    //Drive to short shooting pose and then shooting
    public static Command driveToShortPoseAndShot(RobotContainer robotContainer,Translation2d startingPoint) {
        final SequentialCommandGroup sequence = new SequentialCommandGroup();

        Command moveToShortScoringBalls = robotContainer.driveSubsystem.followPath(
                new Pose2d(startingPoint,Rotation2d.fromDegrees(90)),
                new Translation2d[]{},  //change as the real situation
                new Pose2d(scoreShortBallsPose.getTranslation(),Rotation2d.fromDegrees(0)),
                Rotation2d.fromDegrees(0),
                0.5
         );

        sequence.addCommands(robotContainer.intakeCommand.intakeContinuously()
                .alongWith(robotContainer.shootCommand.fixShootShortContinuously())
                .withTimeout((long)3000));
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

    public static Command driveToIntakeFirstLineContinuousLy(RobotContainer robotContainer){
        final SequentialCommandGroup sequence = new SequentialCommandGroup();
        Command driveAndIntake = robotContainer.driveSubsystem.followStraightLine(
                Positions.LINE_1_RIGHT_BALL,
                Positions.LINE_1_ENDING,
                Rotation2d.fromDegrees(0),
                        0.3
                )
                .alongWith(robotContainer.intakeCommand.intakeContinuously());
        sequence.addCommands(driveAndIntake);

        return sequence;
    }

    public static Command driveToIntakeSecondLineContinuousLy(RobotContainer robotContainer){
        final SequentialCommandGroup sequence = new SequentialCommandGroup();
        Command driveAndIntake = robotContainer.driveSubsystem.followStraightLine(
                        Positions.LINE_2_RIGHT_BALL,
                        Positions.LINE_2_ENDING,
                        Rotation2d.fromDegrees(0),
                        0.3
                )
                .alongWith(robotContainer.intakeCommand.intakeContinuously());
        sequence.addCommands(driveAndIntake);

        return sequence;
    }

}