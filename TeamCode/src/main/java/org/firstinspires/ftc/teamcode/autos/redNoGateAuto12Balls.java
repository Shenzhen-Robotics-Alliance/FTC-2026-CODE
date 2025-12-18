package org.firstinspires.ftc.teamcode.autos;

import static org.firstinspires.ftc.teamcode.autos.AutoUtils.RedScoreShortBallsPose;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.RobotContainer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class redNoGateAuto12Balls implements Auto{

    @Override
    public Command getAutonomousCommands(RobotContainer robotContainer) {
        SequentialCommandGroup sequence = new SequentialCommandGroup();

        // Step 0: Reset Odometry
        sequence.addCommands(new InstantCommand(() -> robotContainer.driveSubsystem.setPose(new Pose2d())));

        // <-- Step 1: Score preloaded Balls and shoot -->
        sequence.addCommands(AutoUtils.RedPreloadDriveToShortPoseAndShot(robotContainer,redPositions.START_POINT));

        // <-- Step 2:  Intake and score the first three Balls -->
        Command driveToFirstLine = robotContainer.driveSubsystem.followPath(
                new Pose2d(RedScoreShortBallsPose.getTranslation(), Rotation2d.fromDegrees(0)),
                new Translation2d[]{new Translation2d(0.81,0.97)},
                new Pose2d(redPositions.LINE_1_RIGHT_BALL,Rotation2d.fromDegrees(90)),
                Rotation2d.fromDegrees(0),
                0.85
        );
        sequence.addCommands(driveToFirstLine.withTimeout(700));
        sequence.addCommands(AutoUtils.RedDriveToIntakeFirstLineContinuousLy(robotContainer).withTimeout(1000));
        sequence.addCommands(AutoUtils.RedFirstLineDriveToShortPoseAndShot(robotContainer, redPositions.LINE_1_ENDING,1300));


        // <-- Step 3:  Intake and score the Second three Balls -->
        Command driveToSecondLine = robotContainer.driveSubsystem.followPath(
                new Pose2d(RedScoreShortBallsPose.getTranslation(),Rotation2d.fromDegrees(0)),
                new Translation2d[]{new Translation2d(0.77,1.1)},
                new Pose2d(redPositions.LINE_2_RIGHT_BALL,Rotation2d.fromDegrees(90)),
                Rotation2d.fromDegrees(0),
                0.85
        );
        Command driveToGATE = robotContainer.driveSubsystem.followPath(
                new Pose2d(redPositions.LINE_2_ENDING,Rotation2d.fromDegrees(60)),
                new Translation2d[]{new Translation2d(0.31,1.36)},
                new Pose2d(redPositions.GATE_POINT,Rotation2d.fromDegrees(90)),
                Rotation2d.fromDegrees(0),
                0.85
        );

        sequence.addCommands(driveToSecondLine.withTimeout(1500));  //narrow
        sequence.addCommands(AutoUtils.RedDriveToIntakeSecondLineContinuousLy(robotContainer).withTimeout(1400));
        sequence.addCommands(AutoUtils.RedSecondLineDriveToShortPoseAndShot(robotContainer,redPositions.LINE_2_ENDING,1900));

        // <-- Step 4:  Intake and score the Third three Balls -->
        Command driveToThirdLine = robotContainer.driveSubsystem.followPath(
                new Pose2d(RedScoreShortBallsPose.getTranslation(),Rotation2d.fromDegrees(0)),
                new Translation2d[]{new Translation2d(0.7,1.46)},   //need
                new Pose2d(redPositions.LINE_3_RIGHT_BALL,Rotation2d.fromDegrees(90)),
                Rotation2d.fromDegrees(0),
                0.85
        );

        sequence.addCommands(driveToThirdLine
                .withTimeout(1800)); //narrow

        sequence.addCommands(AutoUtils.RedDriveToIntakeThirdLineContinuousLy(robotContainer).withTimeout(1400));
        sequence.addCommands(AutoUtils.RedThirdLineDriveToShortPoseAndShot(robotContainer,redPositions.LINE_3_ENDING,2200));

        // <-- Step 5: Go to the Auto Ending Point -->
        Command goToEndingPoint = robotContainer.driveSubsystem.followPath(
                new Pose2d(RedScoreShortBallsPose.getTranslation(),Rotation2d.fromDegrees(0)),
                new Translation2d[]{}, //need
                new Pose2d(redPositions.START_POINT,Rotation2d.fromDegrees(90)),
                Rotation2d.fromRotations(0),
                0.85
        );

        sequence.addCommands(goToEndingPoint.withTimeout(1500));

        return sequence;
    }
}

