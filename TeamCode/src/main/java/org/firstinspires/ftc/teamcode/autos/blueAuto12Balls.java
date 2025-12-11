package org.firstinspires.ftc.teamcode.autos;

import static org.firstinspires.ftc.teamcode.autos.AutoUtils.scoreShortBallsPose;

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
                new Pose2d(scoreShortBallsPose.getTranslation(), Rotation2d.fromDegrees(0)),
                new Translation2d[]{new Translation2d(0.72,-0.78)},
                new Pose2d(bluePositions.LINE_1_RIGHT_BALL,Rotation2d.fromDegrees(90)),
                Rotation2d.fromDegrees(0),
                0.8
        );
        sequence.addCommands(driveToFirstLine
                .withTimeout(900));
        sequence.addCommands(AutoUtils.BlueDriveToIntakeFirstLineContinuousLy(robotContainer).withTimeout(1000));
        sequence.addCommands(AutoUtils.BlueFirstLineDriveToShortPoseAndShot(robotContainer, bluePositions.LINE_1_ENDING,1300));


        // <-- Step 3:  Intake and score the Second three Balls -->
        Command driveToSecondLine = robotContainer.driveSubsystem.followPath(
                new Pose2d(scoreShortBallsPose.getTranslation(),Rotation2d.fromDegrees(0)),
                new Translation2d[]{new Translation2d(1,-1.38)},
                new Pose2d(bluePositions.LINE_2_RIGHT_BALL,Rotation2d.fromDegrees(90)),
                Rotation2d.fromDegrees(0),
                0.8
        );

        sequence.addCommands(driveToSecondLine
                .withTimeout(1400));  //narrow

        sequence.addCommands(AutoUtils.BLueDriveToIntakeSecondLineContinuousLy(robotContainer).withTimeout(1400));
        sequence.addCommands(AutoUtils.BlueSecondLineDriveToShortPoseAndShot(robotContainer,bluePositions.LINE_2_ENDING,2000));

        // <-- Step 4:  Intake and score the Third three Balls -->
        Command driveToThirdLine = robotContainer.driveSubsystem.followPath(
                new Pose2d(scoreShortBallsPose.getTranslation(),Rotation2d.fromDegrees(0)),
                new Translation2d[]{new Translation2d(1,-1.96)},   //need
                new Pose2d(bluePositions.LINE_3_RIGHT_BALL,Rotation2d.fromDegrees(90)),
                Rotation2d.fromDegrees(0),
                0.8
        );

        sequence.addCommands(driveToThirdLine
                .withTimeout(1800)); //narrow

        sequence.addCommands(AutoUtils.BlueDriveToIntakeThirdLineContinuousLy(robotContainer).withTimeout(1400));
        sequence.addCommands(AutoUtils.BlueThirdLineDriveToShortPoseAndShot(robotContainer,bluePositions.LINE_3_ENDING,2000));

        // <-- Step 5: Go to the Auto Ending Point -->
        Command goToEndingPoint = robotContainer.driveSubsystem.followPath(
                new Pose2d(scoreShortBallsPose.getTranslation(),Rotation2d.fromDegrees(0)),
                new Translation2d[]{}, //need
                new Pose2d(bluePositions.START_POINT,Rotation2d.fromDegrees(90)),
                Rotation2d.fromRotations(0),
                0.8
        );

        sequence.addCommands(goToEndingPoint.withTimeout(1500));

        return sequence;
    }
}
