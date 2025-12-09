package org.firstinspires.ftc.teamcode.autos;

import static org.firstinspires.ftc.teamcode.autos.AutoUtils.scoreShortBallsPose;
import static org.firstinspires.ftc.teamcode.autos.AutoUtils.thirdLineDriveToShortPoseAndShot;

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
        sequence.addCommands(AutoUtils.preloadDriveToShortPoseAndShot(robotContainer,bluePositions.START_POINT));

        // <-- Step 2:  Intake and score the first three Balls -->
        Command driveToFirstLine = robotContainer.driveSubsystem.followPath(
                new Pose2d(scoreShortBallsPose.getTranslation(), Rotation2d.fromDegrees(0)),
                new Translation2d[]{new Translation2d(0.72,-0.78)},
                new Pose2d(bluePositions.LINE_1_RIGHT_BALL,Rotation2d.fromDegrees(90)),
                Rotation2d.fromDegrees(0),
                0.9
        );
        sequence.addCommands(driveToFirstLine
                .withTimeout(800));
        sequence.addCommands(AutoUtils.driveToIntakeFirstLineContinuousLy(robotContainer).withTimeout(1000));
        sequence.addCommands(AutoUtils.firstLineDriveToShortPoseAndShot(robotContainer, bluePositions.LINE_1_ENDING,1300));


        // <-- Step 3:  Intake and score the Second three Balls -->
        Command driveToSecondLine = robotContainer.driveSubsystem.followPath(
                new Pose2d(scoreShortBallsPose.getTranslation(),Rotation2d.fromDegrees(0)),
                new Translation2d[]{new Translation2d(1,-1.34)},
                new Pose2d(bluePositions.LINE_2_RIGHT_BALL,Rotation2d.fromDegrees(90)),
                Rotation2d.fromDegrees(0),
                0.9
        );

        sequence.addCommands(driveToSecondLine
                .withTimeout(1000));

        sequence.addCommands(AutoUtils.driveToIntakeSecondLineContinuousLy(robotContainer).withTimeout(1400));
        sequence.addCommands(AutoUtils.secondLineDriveToShortPoseAndShot(robotContainer,bluePositions.LINE_2_ENDING,2000));

        // <-- Step 4:  Intake and score the Third three Balls -->
        Command driveToThirdLine = robotContainer.driveSubsystem.followPath(
                new Pose2d(scoreShortBallsPose.getTranslation(),Rotation2d.fromDegrees(0)),
                new Translation2d[]{},   //need
                new Pose2d(bluePositions.LINE_3_RIGHT_BALL,Rotation2d.fromDegrees(90)),
                Rotation2d.fromDegrees(0),
                0.9
        );

        sequence.addCommands(driveToThirdLine
                .withTimeout(1200));

        sequence.addCommands(AutoUtils.driveToIntakeThirdLineContinuousLy(robotContainer).withTimeout(1400));
        sequence.addCommands(AutoUtils.thirdLineDriveToShortPoseAndShot(robotContainer,bluePositions.LINE_3_ENDING,2000));

        // <-- Step 5: Go to the TeleOp Starting Point -->
        return sequence;
    }
}
