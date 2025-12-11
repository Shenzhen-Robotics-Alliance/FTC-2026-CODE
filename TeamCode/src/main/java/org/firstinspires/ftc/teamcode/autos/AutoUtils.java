//=============AutoUtils==========
package org.firstinspires.ftc.teamcode.autos;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.RobotContainer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class AutoUtils {
    /**
     * BLUE AutoUtils
     */
    public static final Pose2d BlueScoreShortBallsPose = new Pose2d(0.78, -0.49, Rotation2d.fromDegrees(0));
    public static final Pose2d RedScoreShortBallsPose = new Pose2d(0.78, 0.49, Rotation2d.fromDegrees(0));
    public static final Pose2d scoreLongBallsPose = new Pose2d(1.02, -2.67, Rotation2d.fromDegrees(0));
    public static final Pose2d startPose = new Pose2d(0,0,Rotation2d.fromDegrees(0));

    //Drive to short shooting pose and then shooting
    public static Command BluePreloadDriveToShortPoseAndShot(RobotContainer robotContainer, Translation2d startingPoint) {
        final SequentialCommandGroup sequence = new SequentialCommandGroup();

        Command moveToShortScoringBalls = robotContainer.driveSubsystem.followPath(
                new Pose2d(startingPoint,Rotation2d.fromDegrees(135)),
                new Translation2d[]{},
                new Pose2d(BlueScoreShortBallsPose.getTranslation(),Rotation2d.fromDegrees(135)),
                Rotation2d.fromDegrees(0),
                0.75
        ).withTimeout(1080);
        sequence.addCommands(moveToShortScoringBalls);

        sequence.addCommands(robotContainer.intakeCommand.intakeContinuously()
                .alongWith(robotContainer.shootCommand.fixShootShortContinuously())
                .withTimeout(2900));
        sequence.addCommands(robotContainer.shootCommand.shootStop());

        return sequence;
    }

    public static Command BlueFirstLineDriveToShortPoseAndShot(RobotContainer robotContainer, Translation2d startingPoint, long Timeout) {
        final SequentialCommandGroup sequence = new SequentialCommandGroup();

        Command moveToShortScoringBalls = robotContainer.driveSubsystem.followPath(
                new Pose2d(startingPoint,Rotation2d.fromDegrees(45)),
                new Translation2d[]{new Translation2d(0.55,-0.56)},  //change as the real situation
                new Pose2d(BlueScoreShortBallsPose.getTranslation(),Rotation2d.fromDegrees(135)),
                Rotation2d.fromDegrees(0),
                0.75
        ).withTimeout(Timeout);
        sequence.addCommands(moveToShortScoringBalls);

        sequence.addCommands(robotContainer.intakeCommand.intakeContinuously()
                .alongWith(robotContainer.shootCommand.fixShootShortContinuously())
                .withTimeout(2900));
        sequence.addCommands(robotContainer.shootCommand.shootStop());

        return sequence;
    }

    public static Command BlueSecondLineDriveToShortPoseAndShot(RobotContainer robotContainer, Translation2d startingPoint, long Timeout) {
        final SequentialCommandGroup sequence = new SequentialCommandGroup();

        Command moveToShortScoringBalls = robotContainer.driveSubsystem.followPath(
                new Pose2d(startingPoint,Rotation2d.fromDegrees(45)),
                new Translation2d[]{new Translation2d(0.78,-1.21)},  //change as the real situation
                new Pose2d(BlueScoreShortBallsPose.getTranslation(),Rotation2d.fromDegrees(135)),
                Rotation2d.fromDegrees(0),
                0.75
        ).withTimeout(Timeout);
        sequence.addCommands(moveToShortScoringBalls);

        sequence.addCommands(robotContainer.intakeCommand.intakeContinuously()
                .alongWith(robotContainer.shootCommand.fixShootShortContinuously())
                .withTimeout(2900));
        sequence.addCommands(robotContainer.shootCommand.shootStop());

        return sequence;
    }

    public static Command BlueThirdLineDriveToShortPoseAndShot(RobotContainer robotContainer, Translation2d startingPoint, long Timeout) {
        final SequentialCommandGroup sequence = new SequentialCommandGroup();

        Command moveToShortScoringBalls = robotContainer.driveSubsystem.followPath(
                new Pose2d(startingPoint,Rotation2d.fromDegrees(45)),
                new Translation2d[]{},  //in need
                new Pose2d(BlueScoreShortBallsPose.getTranslation(),Rotation2d.fromDegrees(135)),
                Rotation2d.fromDegrees(0),
                0.9
        ).withTimeout(Timeout);
        sequence.addCommands(moveToShortScoringBalls);

        sequence.addCommands(robotContainer.intakeCommand.intakeContinuously()
                .alongWith(robotContainer.shootCommand.fixShootShortContinuously())
                .withTimeout(2900));
        sequence.addCommands(robotContainer.shootCommand.shootStop());

        return sequence;
    }




    //Drive to long shooting pose and then shooting
    public static Command BlueDriveToFarPoseAndShot(RobotContainer robotContainer) {
        final SequentialCommandGroup sequence = new SequentialCommandGroup();

        Command moveToFarScoringBalls = robotContainer.driveSubsystem.followPath(
                new Pose2d(startPose.getTranslation(),Rotation2d.fromDegrees(0)),
                new Translation2d[]{},   //change as the real situation
                new Pose2d(bluePositions.SHOOTING_POINT,Rotation2d.fromDegrees(0)),
                Rotation2d.fromDegrees(0),
                0.7
        );

        sequence.addCommands(moveToFarScoringBalls);

        sequence.addCommands(robotContainer.intakeCommand.intakeContinuously()
                .alongWith(robotContainer.shootCommand.fixShootShortContinuously())
                .withTimeout(3500));
        sequence.addCommands(robotContainer.shootCommand.shootStop());

        return sequence;
    }


    public static Command BlueDriveToIntakeFirstLineContinuousLy(RobotContainer robotContainer){
        final SequentialCommandGroup sequence = new SequentialCommandGroup();
        Command driveAndIntake = robotContainer.driveSubsystem.followPath(
                        new Pose2d(bluePositions.LINE_1_RIGHT_BALL,Rotation2d.fromDegrees(90)),
                        new Translation2d[]{},   //change as the real situation
                        new Pose2d(bluePositions.LINE_1_ENDING,Rotation2d.fromDegrees(90)),
                        Rotation2d.fromDegrees(0),
                        0.7
                )
                .alongWith(robotContainer.intakeCommand.intakeContinuously());
        sequence.addCommands(driveAndIntake);

        return sequence;
    }

    public static Command BLueDriveToIntakeSecondLineContinuousLy(RobotContainer robotContainer){
        final SequentialCommandGroup sequence = new SequentialCommandGroup();
        Command driveAndIntake = robotContainer.driveSubsystem.followPath(
                        new Pose2d(bluePositions.LINE_2_RIGHT_BALL,Rotation2d.fromDegrees(90)),
                        new Translation2d[]{},   //change as the real situation
                        new Pose2d(bluePositions.LINE_2_ENDING,Rotation2d.fromDegrees(90)),
                        Rotation2d.fromDegrees(0),
                        0.7
                )
                .alongWith(robotContainer.intakeCommand.intakeContinuously());
        sequence.addCommands(driveAndIntake);

        return sequence;
    }
    public static Command BlueDriveToIntakeThirdLineContinuousLy(RobotContainer robotContainer){
        final SequentialCommandGroup sequence = new SequentialCommandGroup();
        Command driveAndIntake = robotContainer.driveSubsystem.followPath(
                        new Pose2d(bluePositions.LINE_3_RIGHT_BALL,Rotation2d.fromDegrees(90)),
                        new Translation2d[]{},   //change as the real situation
                        new Pose2d(bluePositions.LINE_3_ENDING,Rotation2d.fromDegrees(90)),
                        Rotation2d.fromDegrees(0),
                        0.7
                )
                .alongWith(robotContainer.intakeCommand.intakeContinuously());
        sequence.addCommands(driveAndIntake);

        return sequence;
    }
    public static Command BlueFarDriveToIntakeThirdLineContinuousLy(RobotContainer robotContainer){
        final SequentialCommandGroup sequence = new SequentialCommandGroup();
        Command driveAndIntake = robotContainer.driveSubsystem.followPath(
                        new Pose2d(bluePositions.FAR_SHOOTING_LINE_3_RIGHT_BALL,Rotation2d.fromDegrees(90)),
                        new Translation2d[]{},   //change as the real situation
                        new Pose2d(bluePositions.FAR_SHOOTING__ENDING_LINE_3_ENDING,Rotation2d.fromDegrees(90)),
                        Rotation2d.fromDegrees(0),
                        0.7
                )
                .alongWith(robotContainer.intakeCommand.intakeContinuously());
        sequence.addCommands(driveAndIntake);

        return sequence;
    }

    /**
     * Red AutoUtils
     */
    public static Command RedPreloadDriveToShortPoseAndShot(RobotContainer robotContainer, Translation2d startingPoint) {
        final SequentialCommandGroup sequence = new SequentialCommandGroup();

        Command moveToShortScoringBalls = robotContainer.driveSubsystem.followPath(
                new Pose2d(startingPoint,Rotation2d.fromDegrees(45)),
                new Translation2d[]{},
                new Pose2d(RedScoreShortBallsPose.getTranslation(),Rotation2d.fromDegrees(45)),
                Rotation2d.fromDegrees(0),
                0.75
        ).withTimeout(1080);
        sequence.addCommands(moveToShortScoringBalls);

        sequence.addCommands(robotContainer.intakeCommand.intakeContinuously()
                .alongWith(robotContainer.shootCommand.fixShootShortContinuously())
                .withTimeout(2900));
        sequence.addCommands(robotContainer.shootCommand.shootStop());

        return sequence;
    }

    public static Command RedFirstLineDriveToShortPoseAndShot(RobotContainer robotContainer, Translation2d startingPoint, long Timeout) {
        final SequentialCommandGroup sequence = new SequentialCommandGroup();

        Command moveToShortScoringBalls = robotContainer.driveSubsystem.followPath(
                new Pose2d(startingPoint,Rotation2d.fromDegrees(135)),
                new Translation2d[]{new Translation2d(0.55,0.56)},  //change as the real situation
                new Pose2d(RedScoreShortBallsPose.getTranslation(),Rotation2d.fromDegrees(45)),
                Rotation2d.fromDegrees(0),
                0.75
        ).withTimeout(Timeout);
        sequence.addCommands(moveToShortScoringBalls);

        sequence.addCommands(robotContainer.intakeCommand.intakeContinuously()
                .alongWith(robotContainer.shootCommand.fixShootShortContinuously())
                .withTimeout(2900));
        sequence.addCommands(robotContainer.shootCommand.shootStop());

        return sequence;
    }

    public static Command RedSecondLineDriveToShortPoseAndShot(RobotContainer robotContainer, Translation2d startingPoint, long Timeout) {
        final SequentialCommandGroup sequence = new SequentialCommandGroup();

        Command moveToShortScoringBalls = robotContainer.driveSubsystem.followPath(
                new Pose2d(startingPoint,Rotation2d.fromDegrees(135)),
                new Translation2d[]{new Translation2d(0.78,1.21)},  //change as the real situation
                new Pose2d(RedScoreShortBallsPose.getTranslation(),Rotation2d.fromDegrees(45)),
                Rotation2d.fromDegrees(0),
                0.75
        ).withTimeout(Timeout);
        sequence.addCommands(moveToShortScoringBalls);

        sequence.addCommands(robotContainer.intakeCommand.intakeContinuously()
                .alongWith(robotContainer.shootCommand.fixShootShortContinuously())
                .withTimeout(2900));
        sequence.addCommands(robotContainer.shootCommand.shootStop());

        return sequence;
    }

    public static Command RedThirdLineDriveToShortPoseAndShot(RobotContainer robotContainer, Translation2d startingPoint, long Timeout) {
        final SequentialCommandGroup sequence = new SequentialCommandGroup();

        Command moveToShortScoringBalls = robotContainer.driveSubsystem.followPath(
                new Pose2d(startingPoint,Rotation2d.fromDegrees(135)),
                new Translation2d[]{},  //in need
                new Pose2d(RedScoreShortBallsPose.getTranslation(),Rotation2d.fromDegrees(45)),
                Rotation2d.fromDegrees(0),
                0.9
        ).withTimeout(Timeout);
        sequence.addCommands(moveToShortScoringBalls);

        sequence.addCommands(robotContainer.intakeCommand.intakeContinuously()
                .alongWith(robotContainer.shootCommand.fixShootShortContinuously())
                .withTimeout(2900));
        sequence.addCommands(robotContainer.shootCommand.shootStop());

        return sequence;
    }




    //Drive to long shooting pose and then shooting
    public static Command RedDriveToFarPoseAndShot(RobotContainer robotContainer) {
        final SequentialCommandGroup sequence = new SequentialCommandGroup();

        Command moveToFarScoringBalls = robotContainer.driveSubsystem.followPath(
                new Pose2d(startPose.getTranslation(),Rotation2d.fromDegrees(0)),
                new Translation2d[]{new Translation2d(0,0)},   //change as the real situation
                new Pose2d(redPositions.SHOOTING_POINT,Rotation2d.fromDegrees(0)),
                Rotation2d.fromDegrees(0),
                0.7
        );

        sequence.addCommands(moveToFarScoringBalls);

        sequence.addCommands(robotContainer.intakeCommand.intakeContinuously()
                .alongWith(robotContainer.shootCommand.fixShootShortContinuously())
                .withTimeout(3500));
        sequence.addCommands(robotContainer.shootCommand.shootStop());

        return sequence;
    }


    public static Command RedDriveToIntakeFirstLineContinuousLy(RobotContainer robotContainer){
        final SequentialCommandGroup sequence = new SequentialCommandGroup();
        Command driveAndIntake = robotContainer.driveSubsystem.followPath(
                        new Pose2d(redPositions.LINE_1_RIGHT_BALL,Rotation2d.fromDegrees(90)),
                        new Translation2d[]{},   //change as the real situation
                        new Pose2d(redPositions.LINE_1_ENDING,Rotation2d.fromDegrees(90)),
                        Rotation2d.fromDegrees(0),
                        0.7
                )
                .alongWith(robotContainer.intakeCommand.intakeContinuously());
        sequence.addCommands(driveAndIntake);

        return sequence;
    }

    public static Command RedDriveToIntakeSecondLineContinuousLy(RobotContainer robotContainer){
        final SequentialCommandGroup sequence = new SequentialCommandGroup();
        Command driveAndIntake = robotContainer.driveSubsystem.followPath(
                        new Pose2d(redPositions.LINE_2_RIGHT_BALL,Rotation2d.fromDegrees(90)),
                        new Translation2d[]{},   //change as the real situation
                        new Pose2d(redPositions.LINE_2_ENDING,Rotation2d.fromDegrees(90)),
                        Rotation2d.fromDegrees(0),
                        0.7
                )
                .alongWith(robotContainer.intakeCommand.intakeContinuously());
        sequence.addCommands(driveAndIntake);

        return sequence;
    }
    public static Command RedDriveToIntakeThirdLineContinuousLy(RobotContainer robotContainer){
        final SequentialCommandGroup sequence = new SequentialCommandGroup();
        Command driveAndIntake = robotContainer.driveSubsystem.followPath(
                        new Pose2d(redPositions.LINE_3_RIGHT_BALL,Rotation2d.fromDegrees(90)),
                        new Translation2d[]{},   //change as the real situation
                        new Pose2d(redPositions.LINE_3_ENDING,Rotation2d.fromDegrees(90)),
                        Rotation2d.fromDegrees(0),
                        0.7
                )
                .alongWith(robotContainer.intakeCommand.intakeContinuously());
        sequence.addCommands(driveAndIntake);

        return sequence;
    }
}






