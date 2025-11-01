package org.firstinspires.ftc.teamcode.subsystems.SuperStructure;

import static org.firstinspires.ftc.teamcode.constants.SystemConstants.telemetry;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RotSubsystem extends SubsystemBase {
    private static final double SOFT_LIMIT = 800;
    private static final double RETURN_SPEED = 1;
    public final LinearMotion rotateMotion;

    public RotSubsystem(HardwareMap hardwareMap){
        this.rotateMotion = new LinearMotion(
                "Rotate",
                new DcMotorEx[]{hardwareMap.get(DcMotorEx.class,"Rot")},
                new boolean[]{false},
                hardwareMap.get(DcMotorEx.class,"Rot"),
                false,
                2000,
                0.35,
                0,
                0.006,
                0
        );
    }

    public void periodic() {
        rotateMotion.periodic();
        telemetry.addData("=========Rotate=========", "");
        telemetry.addData("Target Vel", "%.0f RPM", rotateMotion.getTargetVelocity()); //* 2000
        telemetry.addData("Current Vel", "%.0f RPM", rotateMotion.getCurrentVelocityRaw());
        telemetry.addData("Vel Error", "%.0f RPM", rotateMotion.getVelocityError()); //* 2000
        telemetry.addData("Output Power", "%.3f", rotateMotion.getOutputPower());
        telemetry.addData("Encoder Pos", "%.0f", rotateMotion.getPosition());
        double pos = rotateMotion.getPosition();
        double targetVel = rotateMotion.getTargetVelocity();


        if (Math.abs(pos) > SOFT_LIMIT) {
            if (pos < -SOFT_LIMIT && targetVel > 0) {
                rotateMotion.runPower(0);
            } else if (pos > SOFT_LIMIT && targetVel < 0) {
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
    public void gotoPosition(double setpoint){
        rotateMotion.goToPosition(setpoint);
    }

}
