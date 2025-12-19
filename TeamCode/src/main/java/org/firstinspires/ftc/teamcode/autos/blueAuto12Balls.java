package org.firstinspires.ftc.teamcode.autos;

import static org.firstinspires.ftc.teamcode.autos.AutoUtils.BlueScoreShortBallsPose;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.RobotContainer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class blueAuto12Balls implements Auto{

    @Override
    public Command getAutonomousCommands(RobotContainer robotContainer) {
        SequentialCommandGroup sequence = new SequentialCommandGroup();

        // Step 0: Reset Odometry
        sequence.addCommands(new InstantCommand(() -> robotContainer.driveSubsystem.setPose(new Pose2d())));

        // <-- Step 1: Score preloaded Balls and shoot -->
        sequence.addCommands(AutoUtils.BluePreloadDriveToShortPoseAndShot(robotContainer,bluePositions.START_POINT));

        // <-- Step 2:  Intake and score the first three Balls -->
        Command driveToFirstLine = robotContainer.driveSubsystem.followPath(
                new Pose2d(BlueScoreShortBallsPose.getTranslation(), Rotation2d.fromDegrees(90)),
                new Translation2d[]{new Translation2d(0.72,-0.78)},
                new Pose2d(bluePositions.LINE_1_RIGHT_BALL,Rotation2d.fromDegrees(0)),
                Rotation2d.fromDegrees(0),
                0.9
        );
        sequence.addCommands(driveToFirstLine
                .withTimeout(700));
        sequence.addCommands(AutoUtils.BlueDriveToIntakeFirstLineContinuousLy(robotContainer).withTimeout(900));
        sequence.addCommands(AutoUtils.BlueFirstLineDriveToShortPoseAndShot(robotContainer, bluePositions.LINE_1_ENDING,1100));


        // <-- Step 3:  Intake and score the Second three Balls -->
        Command driveToSecondLine = robotContainer.driveSubsystem.followPath(
                new Pose2d(BlueScoreShortBallsPose.getTranslation(),Rotation2d.fromDegrees(160)),
                new Translation2d[]{new Translation2d(0.85,-1.07)},
                new Pose2d(bluePositions.LINE_2_RIGHT_BALL,Rotation2d.fromDegrees(80)),
                Rotation2d.fromDegrees(0),
                0.9
        );
        Command driveToGATE = robotContainer.driveSubsystem.followPath(
                new Pose2d(bluePositions.LINE_2_ENDING,Rotation2d.fromDegrees(60)),
                new Translation2d[]{new Translation2d(0.31,-1.36)},
                new Pose2d(bluePositions.GATE_POINT,Rotation2d.fromDegrees(90)),
                Rotation2d.fromDegrees(0),
                0.9
        );

        sequence.addCommands(driveToSecondLine.withTimeout(1350));  //narrow
        sequence.addCommands(AutoUtils.BLueDriveToIntakeSecondLineContinuousLy(robotContainer).withTimeout(1600));

        sequence.addCommands(driveToGATE.withTimeout(1000));  //narrow
        sequence.addCommands(AutoUtils.BlueSecondLineDriveToShortPoseAndShot(robotContainer,bluePositions.GATE_POINT,1800));

        // <-- Step 4:  Intake and score the Third three Balls -->
        Command driveToThirdLine = robotContainer.driveSubsystem.followPath(
                new Pose2d(BlueScoreShortBallsPose.getTranslation(),Rotation2d.fromDegrees(135)),
                new Translation2d[]{new Translation2d(0.96,-2.0)},   //need
                new Pose2d(bluePositions.LINE_3_RIGHT_BALL,Rotation2d.fromDegrees(75)),
                Rotation2d.fromDegrees(0),
                0.9
        );

        sequence.addCommands(driveToThirdLine
                .withTimeout(2100)); //narrow

        sequence.addCommands(AutoUtils.BlueDriveToIntakeThirdLineContinuousLy(robotContainer).withTimeout(1700));
        sequence.addCommands(AutoUtils.BlueThirdLineDriveToShortPoseAndShot(robotContainer,bluePositions.LINE_3_ENDING,2300));

        // <-- Step 5: Go to the Auto Ending Point -->
        Command goToEndingPoint = robotContainer.driveSubsystem.followPath(
                new Pose2d(BlueScoreShortBallsPose.getTranslation(),Rotation2d.fromDegrees(0)),
                new Translation2d[]{}, //need
                new Pose2d(bluePositions.START_POINT,Rotation2d.fromDegrees(90)),
                Rotation2d.fromRotations(0),
                0.9
        );

        sequence.addCommands(goToEndingPoint.withTimeout(1500));

        return sequence;
    }
}
