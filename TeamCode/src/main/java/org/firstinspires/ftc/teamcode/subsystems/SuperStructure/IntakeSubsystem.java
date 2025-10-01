package org.firstinspires.ftc.teamcode.subsystems.SuperStructure;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import edu.wpi.first.math.controller.PIDController;

public class IntakeSubsystem extends SubsystemBase{

    public DcMotorEx intakeMotor;
    private Telemetry telemetry;
    private DcMotor encoder;
    public static double Kp = 0, Ki = 0, Kd = 0, Kf = 0;
    private double integralLimit = 0.3;
    public final double maximumPower =1;
    public final double minimumPower = -1;
    public  double targetVelocity = 0 ;

    public static final double INTAKE_SPEED = 1800;   // intake speed (ticks/sec)
    public static final double OUTTAKE_SPEED = -1600;  // outtake speed (ticks/sec)
    public static final double STOP_SPEED = 0;        // stop


    //predicted

    private PIDController velocityController;

    public void init(HardwareMap hardwareMap,double kf){
        this.intakeMotor = hardwareMap.get(DcMotorEx.class,"intakeMotor");
        this.encoder = hardwareMap.get(DcMotor.class,"intakeMotor");
        this.intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        this.intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.encoder = hardwareMap.get(DcMotor.class,"intakeMotor");

        this.velocityController = new PIDController(Kp,Ki,Kd);
        this.Kf = kf;
    }


    public void periodic(){
        updatePidControl();
    }

    public void updatePidControl(){
        double currentVelocity = intakeMotor.getVelocity();

        double pidOutput = velocityController.calculate(currentVelocity,targetVelocity);

        double feedforward = Kf * targetVelocity;

        //totalOutput = feedforward + pidOutput
        double totalOutput = pidOutput + feedforward;

        //restrict the range of output
        totalOutput = Range.clip(totalOutput,minimumPower,maximumPower);

        intakeMotor.setPower(totalOutput);
    }

    public void intakeStop(){
        intakeMotor.setPower(0);
    }

    public void setTargetVelocity(double velocity) {
        // reset pid parameters when face giant changes
        if (Math.abs(velocity - targetVelocity) > Math.abs(targetVelocity) * 0.5) {
            velocityController.reset();
        }
        this.targetVelocity = velocity;
    }

    // get current velocity
    public double getCurrentVelocity() {
        return intakeMotor.getVelocity();
    }

    //get target velocity
    public double getTargetVelocity() {
        return targetVelocity;
    }

    //get errors
    public double getError() {
        return targetVelocity - getCurrentVelocity();
    }

    // ====== PID parameter management ======
    public void setPIDParameters(double kP, double kI, double kD, double kF) {
        velocityController.setPID(kP, kI, kD);
        this.Kf = kF;
    }

    public void resetPID() {
        velocityController.reset();
    }
}