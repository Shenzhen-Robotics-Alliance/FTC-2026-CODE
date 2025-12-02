package org.firstinspires.ftc.teamcode.subsystems.SuperStructure;
//2,=========shooterSubsystem==========
import static org.firstinspires.ftc.teamcode.constants.SystemConstants.telemetry;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ShooterSubsystem extends SubsystemBase {
    public final LinearMotion shooter;

    private final double MOTOR_CPR = 104; //counts per revolution
    private final double MAX_TICKS_PER_SEC = 1500;
    private final double TOLERANCE_RPM = 50;
    private final double TOLERANCE_TICKS = (TOLERANCE_RPM * MOTOR_CPR) / 60.0;
    private double targetTPS = 0;
    private double currentTPS = 0;
    private double MotorScale = 2.3;

    public ShooterSubsystem(HardwareMap hardwareMap){
        this.shooter = new LinearMotion(
                "shooter",
                new DcMotorEx[]{
                        hardwareMap.get(DcMotorEx.class,"ShooterMotor"),
                },
                new boolean[]{true},
                hardwareMap.get(DcMotorEx.class,"ShooterMotor"),
                false,
                MAX_TICKS_PER_SEC,
                1,
                0,
                0.004,
                0.0006        
        );
    }

    public void periodic() {
        shooter.periodic();
        telemetry.addData("=== SHOOTER ===", "");
        telemetry.addData("Target Vel", "%.0f RPM", shooter.getTargetVelocity());
        telemetry.addData("Vel TPS", "%.0f tick/s", this.targetTPS);
        telemetry.addData("Current TPS", "%.0f tick/s", this.currentTPS);
        telemetry.addData("Encoder Pos", "%.0f", shooter.getPosition());
        telemetry.addData("===== SHOOTER STATUS =====", "");
        telemetry.addData("ready to short shoot?",isReadyToFixShortLaunch());
        telemetry.addData("ready to far shoot?",isReadyToFixFarLaunch());

    }

    //<General Functions for the motor control>
    public void setTargetRPM(double rpm) {
        //limit control motor
        double clampedRPM = Math.min(rpm, 1500);
        //Translate rpm to Ticks/Sec
        targetTPS = (clampedRPM * MOTOR_CPR) / 60.0;

        //normalized to (0.0 - 1.0)
        double normalized =targetTPS*MotorScale/ MAX_TICKS_PER_SEC;

        shooter.setTargetVelocity(Math.max(0, normalized));
    }

    

    public boolean isAtTargetSpeed() {
        // get Ticks/Sec
        currentTPS = shooter.getCurrentVelocityRaw();
        return Math.abs(currentTPS - targetTPS) < TOLERANCE_TICKS;
    }

    

    public Command setShootingMotorStop(){
        return new RunCommand(() -> shooter.setTargetVelocity(0));
    }


    //<Fixed Point shoot in both Short and Far Point>
    public Command shooterFixFarLaunch(){
        return new RunCommand(() -> setTargetRPM(1200));
    }

    public Command shooterFixShortLaunch(){
        return new RunCommand(() -> setTargetRPM(1000));
    }

    public boolean isReadyToFixFarLaunch(){
        currentTPS = shooter.getCurrentVelocityRaw();
        return currentTPS > 1100;
    }

    public boolean isReadyToFixShortLaunch() {
        currentTPS = shooter.getCurrentVelocityRaw();
        return currentTPS > 900;
    }
}
