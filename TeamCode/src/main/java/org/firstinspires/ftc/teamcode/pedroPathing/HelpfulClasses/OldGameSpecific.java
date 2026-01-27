package org.firstinspires.ftc.teamcode.pedroPathing.HelpfulClasses;

import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.PID;

public class OldGameSpecific {
    private CRServoImplEx CR1, CR2, CR0;
    private DcMotorEx intake, flywheel1, flywheel2;

    private ElapsedTime stateTimer = new ElapsedTime();
    private enum FlywheelState{
        IDLE,
        SPIN_UP,
        LAUNCH,
        RESET
    }
    private FlywheelState flywheelState = FlywheelState.IDLE;

    VelocityPID fly1_PID = new VelocityPID(PID.fly1P, PID.fly1I, PID.fly1D, PID.fly1F, true);
    VelocityPID fly2_PID = new VelocityPID(PID.fly2P, PID.fly2I, PID.fly2D, PID.fly2F, true);

    private int shotRemaining = 0;
    private final double minFlywheelSpeed = 600;
    private final double closeTarget = 750;
    private final double farTarget = 925;
    private boolean far;
    private final double FLYWHEEL_MAX_SPINUP_TIME = 1.5;

    public void init(HardwareMap hardwareMap){
        CR1 = hardwareMap.get(CRServoImplEx.class, "CR1");
        CR0 = hardwareMap.get(CRServoImplEx.class, "CR0");
        CR2 = hardwareMap.get(CRServoImplEx.class, "CR2");

        CR0.setDirection(CRServoImplEx.Direction.FORWARD);
        CR1.setDirection(CRServoImplEx.Direction.FORWARD);
        CR2.setDirection(CRServoImplEx.Direction.FORWARD);

        intake = (DcMotorEx) hardwareMap.dcMotor.get("intake");
        flywheel1 = (DcMotorEx) hardwareMap.dcMotor.get("fly1");
        flywheel2 = (DcMotorEx) hardwareMap.dcMotor.get("fly2");

        flywheel1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        flywheel2.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheel1.setDirection(DcMotorSimple.Direction.REVERSE);

        flywheel2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        CR0.setPower(0);
        CR1.setPower(0);
        CR2.setPower(0);

        intake.setPower(0);
        flywheel1.setPower(0);
        flywheel2.setPower(0);

        flywheelState = FlywheelState.IDLE;
        shotRemaining = 0;
        stateTimer.reset();
    }

    public void update(){
        switch(flywheelState){
            case IDLE:
                if (shotRemaining > 0){
                    if (far){
//                        flywheel1.setPower(fly1_PID.update(farTarget, flywheel1.getVelocity()));
//                        flywheel2.setPower(fly1_PID.update(farTarget, flywheel1.getVelocity()));
                    }
                    else if (!far){
//                        flywheel1.setPower(fly1_PID.update(closeTarget, flywheel1.getVelocity()));
//                        flywheel2.setPower(fly1_PID.update(closeTarget, flywheel1.getVelocity()));
                    }

                    CR0.setPower(0);
                    stateTimer.reset();
                    flywheelState = FlywheelState.SPIN_UP;
                }
                break;

            case SPIN_UP:
                if (far){
//                    flywheel1.setPower(fly1_PID.update(farTarget, flywheel1.getVelocity()));
//                    flywheel2.setPower(fly1_PID.update(farTarget, flywheel1.getVelocity()));
                }
                else if (!far){
//                    flywheel1.setPower(fly1_PID.update(closeTarget, flywheel1.getVelocity()));
//                    flywheel2.setPower(fly1_PID.update(closeTarget, flywheel1.getVelocity()));
                }

                if (((flywheel1.getVelocity() > minFlywheelSpeed) || (flywheel2.getVelocity() > minFlywheelSpeed) ||
                        (stateTimer.seconds() > FLYWHEEL_MAX_SPINUP_TIME)) && (stateTimer.seconds() > 0.5)){
                    CR0.setPower(1);
                    CR2.setPower(1);
                    CR1.setPower(1);
                    intake.setPower(0.5);
                    stateTimer.reset();
                    flywheelState = FlywheelState.LAUNCH;
                }
                break;

            case LAUNCH:
                if (far){
//                    flywheel1.setPower(fly1_PID.update(farTarget, flywheel1.getVelocity()));
//                    flywheel2.setPower(fly1_PID.update(farTarget, flywheel1.getVelocity()));
                }
                else if (!far){
//                    flywheel1.setPower(fly1_PID.update(closeTarget, flywheel1.getVelocity()));
//                    flywheel2.setPower(fly1_PID.update(closeTarget, flywheel1.getVelocity()));
                }

                if (stateTimer.seconds() > 4 ||
                        ((flywheel1.getCurrent(CurrentUnit.AMPS) > 6) && flywheel1.getVelocity() > 500 && stateTimer.seconds() > 0.4) ||
                        (flywheel2.getCurrent(CurrentUnit.AMPS) > 6 && flywheel1.getVelocity() > 500 && stateTimer.seconds() > 0.4)){
                    shotRemaining--;
                    CR0.setPower(0);
                    CR1.setPower(0);
                    CR2.setPower(0);
                    intake.setPower(0);
                    flywheelState = FlywheelState.RESET;
                }
                break;

            case RESET:
                if (shotRemaining > 0){
                    if (far){
//                        flywheel1.setPower(fly1_PID.update(farTarget, flywheel1.getVelocity()));
//                        flywheel2.setPower(fly1_PID.update(farTarget, flywheel1.getVelocity()));
                    }
                    else if (!far){
//                        flywheel1.setPower(fly1_PID.update(closeTarget, flywheel1.getVelocity()));
//                        flywheel2.setPower(fly1_PID.update(closeTarget, flywheel1.getVelocity()));
                    }
                    CR0.setPower(0);

                    stateTimer.reset();
                    flywheelState = FlywheelState.SPIN_UP;
                }
                else{
                    flywheel1.setPower(0);
                    flywheel2.setPower(0);
                    CR0.setPower(0);
                    CR1.setPower(0);
                    CR2.setPower(0);
                    flywheelState = FlywheelState.IDLE;
                }

                break;
        }
    }

    public void fireShots(int shots, boolean isfar) {
        if (flywheelState == FlywheelState.IDLE && shots > 0) {
            far = isfar;
            shotRemaining = shots;
            stateTimer.reset();
            flywheelState = FlywheelState.SPIN_UP;
        }
    }

    public boolean isShooting(){
        return (flywheelState != FlywheelState.IDLE);
    }

    public void powerIntake(){
        intake.setPower(0.5);
        CR0.setPower(1);
        CR1.setPower(1);
        CR2.setPower(1);
    }

    public void powerIntakeButNotTheLastOne(){
        intake.setPower(0.5);
        CR1.setPower(1);
        CR2.setPower(1);
    }

    public void intakeOff(){
        intake.setPower(0);
        CR0.setPower(0);
        CR1.setPower(0);
        CR2.setPower(0);
    }


}
