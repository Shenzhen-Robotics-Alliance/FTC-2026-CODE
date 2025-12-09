package org.firstinspires.ftc.teamcode.codeEntrances;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutonomousRobot;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.autos.blueAutoThreePlusNineBalls;
import org.firstinspires.ftc.teamcode.autos.blueAutoThreePlusNineBalls;
import org.firstinspires.ftc.teamcode.autos.blueAutoThreePlusNineBalls;
import org.firstinspires.ftc.teamcode.utils.AllianceSide;
import org.firstinspires.ftc.teamcode.utils.OpModeUtils;

@Autonomous(name="<Auto> BLUE 3 + 6 score artifacts ")
public class ShortPoint9score extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        final AutonomousRobot robot = new AutonomousRobot(
                new RobotContainer(hardwareMap, AllianceSide.BLUE),
                new blueAutoThreePlusNineBalls()
        );
        OpModeUtils.runAutoMode(robot, this);
    }
}