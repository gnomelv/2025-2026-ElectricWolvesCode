package org.firstinspires.ftc.teamcode.pedroPathing.Tests;


import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.HelpfulClasses.VelocityPID;
import org.firstinspires.ftc.teamcode.pedroPathing.PID;

@TeleOp(name = "Motor Test", group = "Test")
public class Motor_Test extends OpMode {
    DcMotorEx intake;
    DcMotorEx flywheel1;
    DcMotorEx flywheel2;
    private CRServoImplEx CR1, CR2, CR0;
    TelemetryManager telemetryM;

    double speedfactor;

    VelocityPID fly1_PID = new VelocityPID(PID.fly1P, PID.fly1I, PID.fly1D, PID.fly1F, true);
    VelocityPID fly2_PID = new VelocityPID(PID.fly2P, PID.fly2I, PID.fly2D, PID.fly2F, true);

    @Override
    public void init() {
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

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        CR0.setPower(0);
        CR1.setPower(0);
        CR2.setPower(0);

        intake.setPower(0);
        flywheel1.setPower(0);
        flywheel2.setPower(0);
    }


//2333
    @Override
    public void loop() {
        if (gamepad1.x){
//            flywheel1.setPower(fly1_PID.update(750, flywheel1.getVelocity()));
//            flywheel2.setPower(fly2_PID.update(750, flywheel2.getVelocity()));
        }
        else{
            flywheel1.setPower(0);
            flywheel2.setPower(0);
        }
        if (gamepad1.y){
            CR0.setPower(1);
            CR1.setPower(1);
            CR2.setPower(1);
        }

        telemetryM.addData("FlY 1 Current", (flywheel1.getCurrent(CurrentUnit.AMPS)));
        telemetryM.addData("FlY 2 Current", (flywheel2.getCurrent(CurrentUnit.AMPS)));
        telemetryM.update();

    }
}


