package org.firstinspires.ftc.teamcode.subsystems.SuperStructure;

import static org.firstinspires.ftc.teamcode.constants.SystemConstants.ROT_SOFT_LIMIT;
import static org.firstinspires.ftc.teamcode.constants.SystemConstants.telemetry;
import static org.firstinspires.ftc.teamcode.constants.SystemConstants.ROT_INIT_POSITION;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RotSubsystem extends SubsystemBase {
    private static final double RETURN_SPEED = 1;
    public final LinearMotion rotateMotion;

    public RotSubsystem(HardwareMap hardwareMap){
        this.rotateMotion = new LinearMotion(
                "Rotate",
                new DcMotorEx[]{hardwareMap.get(DcMotorEx.class,"Rot")},
                new boolean[]{false},
                hardwareMap.get(DcMotorEx.class,"Rot"),
                false,
                2500,
                0.25,
                0,
                0.005,
                0
        );
    }


    public void periodic() {
        rotateMotion.periodic();
        telemetry.addData("=========Rotate=========", "");
        telemetry.addData("Target Vel", "%.0f RPM", rotateMotion.getTargetVelocity()); //* 2000
        telemetry.addData("Current Vel", "%.0f RPM", rotateMotion.getCurrentVelocityRaw());
        telemetry.addData("Vel Error", "%.0f RPM", rotateMotion.getVelocityError()); //* 2000
        telemetry.addData("Encoder Pos", "%.0f", rotateMotion.getPosition());
        telemetry.addData("init position", "%.0f", rotateMotion.getCurrentSetPoint());
        double pos = rotateMotion.getPosition();
        double targetVel = rotateMotion.getTargetVelocity();

        if (Math.abs(pos) > ROT_SOFT_LIMIT+ROT_INIT_POSITION) {
            if (pos < -ROT_SOFT_LIMIT+ROT_INIT_POSITION && targetVel > 0) {
                rotateMotion.runPower(0);
            } else if (pos > ROT_SOFT_LIMIT+ROT_INIT_POSITION && targetVel < 0) {
                rotateMotion.runPower(0);
            } else {
                rotateMotion.setTargetVelocity(
                        pos > 0 ? -RETURN_SPEED : RETURN_SPEED
                );

            }
        }
    }


    public void setRotateStop(){
        rotateMotion.setMotorsStop();
        rotateMotion.resetController();
    }

    public void setRotateVelocity(double velocity) {
        rotateMotion.setTargetVelocity(velocity);
    }

    public void setManualRotPower(double power){
        rotateMotion.runPower(power);
    }

    public double getCurrentRotateVelocity() {
        return rotateMotion.getCurrentVelocity();
    }

    public double getTargetRotateVelocity() {
        return rotateMotion.getTargetVelocity();
    }
    public double getCurrentPosition(){
        return rotateMotion.getPosition();
    }


}
