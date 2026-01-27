package org.firstinspires.ftc.teamcode.pedroPathing.Tests;


import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.pedroPathing.HelpfulClasses.VelocityPID;
import org.firstinspires.ftc.teamcode.pedroPathing.PID;
@Disabled
@TeleOp(name = "Meet 1 Testing", group = "Test")
public class Meet_1_Testing extends OpMode {

    TelemetryManager telemetryM;
    DcMotorEx intake, flywheel1, flywheel2, frontLeft, frontRight, backLeft, backRight;
    CRServoImplEx CR1, CR2, CR0;
    VelocityPID fly1_PID = new VelocityPID(PID.fly1P, PID.fly1I, PID.fly1D, PID.fly1F,true);
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

        flywheel1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheel2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        flywheel2.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheel1.setDirection(DcMotorSimple.Direction.REVERSE);

        flywheel2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();


    }

    @Override
    public void loop() {

//        flywheel1.setPower(fly1_PID.update(1100, flywheel1.getVelocity()));
//        flywheel2.setPower(fly2_PID.update(1100, flywheel2.getVelocity()));
        telemetryM.addData("FlY 1 Velocity", (flywheel1.getVelocity()));
        telemetryM.addData("FlY 2 Velocity", (flywheel2.getVelocity()));
        telemetryM.addData("Target Velocity", 1100);
        telemetryM.update();


        //Transfer Control
        if (gamepad1.a) {
            CR0.setPower(1);
            CR1.setPower(1);
            CR2.setPower(1);
        }
        else if (gamepad1.b) {
            CR0.setPower(-1);
            CR1.setPower(-1);
            CR2.setPower(-1);
        }
        else {
            CR0.setPower(0);
            CR1.setPower(0);
            CR2.setPower(0);
        }

        //Outtake Control
        flywheel1.setPower(fly1_PID.update(1100, flywheel1.getVelocity(),0));
        flywheel2.setPower(fly2_PID.update(1100, flywheel2.getVelocity(),0));

        if (gamepad1.y) {
            flywheel1.setPower(fly1_PID.update(-1100, flywheel1.getVelocity(),0));
            flywheel2.setPower(fly2_PID.update(-1100, flywheel2.getVelocity(),0));
        }
//        else {
//            flywheel1.setPower(0);
//            flywheel2.setPower(0);
//        }

        //Intake Control
        if (gamepad1.right_bumper){
            intake.setPower(1);
        }
        else if (gamepad1.left_bumper){
            intake.setPower(-1);
        }
        else {
            intake.setPower(0);
        }


    }
}