package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.commands.drive.JoystickDriveFactory;
import org.firstinspires.ftc.teamcode.utils.MapleJoystickDriveInput;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * robot during teleop stage'
 * here we bind all the commands to a key on the GamePad
 * */
public class TeleOpRobot extends Robot {
    private final RobotContainer robotContainer;
    private final GamepadEx pilotGamePad, copilotGamePad;
    private final Runnable calibrateOdometry;
    public TeleOpRobot(RobotContainer robotContainer, Gamepad pilotGamePad, Gamepad copilotGamePad) {
        super();
        this.robotContainer = robotContainer;
        this.pilotGamePad = new GamepadEx(pilotGamePad);
        this.copilotGamePad = new GamepadEx(copilotGamePad);

        this.calibrateOdometry = () -> robotContainer.driveSubsystem.setPose(new Pose2d());
        calibrateOdometry.run();
        configureKeyBindings();
    }

    private void configureKeyBindings() {
        this.pilotGamePad.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON).whenHeld(JoystickDriveFactory.joystickDrive(
                robotContainer.driveSubsystem,
                MapleJoystickDriveInput.leftHandedJoystick(pilotGamePad),
                () -> true));

        robotContainer.driveSubsystem.setDefaultCommand(JoystickDriveFactory.joystickDrive(
                robotContainer.driveSubsystem,
                MapleJoystickDriveInput.leftHandedJoystick(pilotGamePad),
                () -> true));

        this.pilotGamePad.getGamepadButton(GamepadKeys.Button.START).whenPressed(calibrateOdometry);

        //pilot left trigger to intake
        if(pilotGamePad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5 && robotContainer.intakeContinueCommand.isFinished()){
            robotContainer.intakeContinueCommand.execute();
        }

        //pilot A stop the intake
        if(pilotGamePad.getButton(GamepadKeys.Button.A)){
            robotContainer.intakeStop.end(true);
        }

        //pilot left bumper outtake
        if (pilotGamePad.getButton(GamepadKeys.Button.LEFT_BUMPER) && robotContainer.outtakeContinueCommand.isFinished()){
            robotContainer.outtakeContinueCommand.execute();
        }

        //copilot LeftX control the shooter rotate
        if(copilotGamePad.getLeftX()>0.1){
            robotContainer.rotSubsystem.setTargetVelocity(0.5);
            robotContainer.rotCommands.execute();
        }

        //copilot A stop shooter rotate
        if(copilotGamePad.getButton(GamepadKeys.Button.A)){
            robotContainer.rotSubsystem.setRotStop();
        }

    }


    @Override
    public void reset() {
        super.reset();
        try {
            robotContainer.close();
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }
}
