package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.commands.drive.JoystickDriveCommandFactory;
import org.firstinspires.ftc.teamcode.utils.MapleJoystickDriveInput;

import java.io.IOException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * robot during teleop stage'
 * here we bind all the commands to a key on the GamePad
 * */
public class TeleOpRobot extends Robot {
    private final RobotContainer robotContainer;
    private final GamepadEx pilotGamePad, copilotGamePad;
    private final Runnable calibrateIMU;
    public TeleOpRobot(RobotContainer robotContainer, Gamepad pilotGamePad, Gamepad copilotGamePad) {
        super();
        this.robotContainer = robotContainer;
        this.pilotGamePad = new GamepadEx(pilotGamePad);
        this.copilotGamePad = new GamepadEx(copilotGamePad);

        this.calibrateIMU = () -> robotContainer.driveSubsystem.setPose(new Pose2d(
                robotContainer.driveSubsystem.getPose().getTranslation(),
                new Rotation2d()
        ));
        calibrateIMU.run();
        configureKeyBindings();
    }

    private void configureKeyBindings() {
        robotContainer.driveSubsystem.setDefaultCommand(JoystickDriveCommandFactory.joystickDrive(
                robotContainer.driveSubsystem,
                MapleJoystickDriveInput.leftHandedJoystick(pilotGamePad),
                () -> pilotGamePad.gamepad.right_bumper
        ));

        this.pilotGamePad.getGamepadButton(GamepadKeys.Button.START).whenPressed(calibrateIMU);
    }

    @Override
    public void reset() {
        super.reset();
        try {
            robotContainer.close();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }
}
