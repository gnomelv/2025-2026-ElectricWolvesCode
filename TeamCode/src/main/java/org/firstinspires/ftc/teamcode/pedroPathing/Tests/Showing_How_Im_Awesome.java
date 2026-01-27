package org.firstinspires.ftc.teamcode.pedroPathing.Tests;


import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.HelpfulClasses.VelocityPID;
import org.firstinspires.ftc.teamcode.pedroPathing.PID;

@TeleOp(name = "SHOWING THAT THE PID IS DOPE AS FREAK")
public class Showing_How_Im_Awesome extends OpMode {
    TelemetryManager telemetryM;
    private DcMotorEx flywheel1, flywheel2;
    private LynxModule hub;
    VelocityPID fly1_PID = new VelocityPID(PID.fly1P, PID.fly1I, PID.fly1D, PID.fly1F, true);
    VelocityPID fly2_PID = new VelocityPID(PID.fly2P, PID.fly2I, PID.fly2D, PID.fly2F,true);

    @Override
    public void init() {
        hub = hardwareMap.getAll(LynxModule.class).get(0);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        flywheel1 = (DcMotorEx) hardwareMap.dcMotor.get("fly1");
        flywheel2 = (DcMotorEx) hardwareMap.dcMotor.get("fly2");

        flywheel1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        flywheel2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        flywheel2.setDirection(DcMotorEx.Direction.REVERSE);
        flywheel1.setDirection(DcMotorEx.Direction.FORWARD);

        flywheel1.setVelocityPIDFCoefficients(0,0,0,0);
        flywheel2.setVelocityPIDFCoefficients(0,0,0,0);

        flywheel2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        flywheel1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        flywheel1.setPower(0);
        flywheel2.setPower(0);

    }

    @Override
    public void loop(){
        if (gamepad1.right_bumper){
            flywheel2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            flywheel1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }
        else if (gamepad1.left_bumper){
            flywheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            flywheel2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }



       //MY OWN
        if (gamepad1.a){
            flywheel1.setPower(fly1_PID.update(500,flywheel1.getVelocity(), hub.getInputVoltage(VoltageUnit.VOLTS)));
            flywheel2.setPower(fly2_PID.update(500,flywheel2.getVelocity(), hub.getInputVoltage(VoltageUnit.VOLTS)));
            telemetryM.addData("Target Speed", 500);
        }
        else if (gamepad1.b){
            flywheel1.setPower(fly1_PID.update(650,flywheel1.getVelocity(), hub.getInputVoltage(VoltageUnit.VOLTS)));
            flywheel2.setPower(fly2_PID.update(650,flywheel2.getVelocity(), hub.getInputVoltage(VoltageUnit.VOLTS)));
            telemetryM.addData("Target Speed", 650);

        }
        else if (gamepad1.y){
            flywheel1.setPower(fly1_PID.update(800,flywheel1.getVelocity(), hub.getInputVoltage(VoltageUnit.VOLTS)));
            flywheel2.setPower(fly2_PID.update(800,flywheel2.getVelocity(), hub.getInputVoltage(VoltageUnit.VOLTS)));
            telemetryM.addData("Target Speed", 800);

        }
        else if (gamepad1.x){
            flywheel1.setPower(fly1_PID.update(1000,flywheel1.getVelocity(), hub.getInputVoltage(VoltageUnit.VOLTS)));
            flywheel2.setPower(fly2_PID.update(1000,flywheel2.getVelocity(), hub.getInputVoltage(VoltageUnit.VOLTS)));
            telemetryM.addData("Target Speed", 1000);

        }


//BUILT IN
        else if (gamepad1.dpad_down){
            flywheel1.setVelocity(500/28, AngleUnit.DEGREES);
            flywheel1.setVelocity(500/28, AngleUnit.DEGREES);
            telemetryM.addData("Target Speed", 500);

        }
        else if (gamepad1.dpad_right){
            flywheel1.setVelocity(650/28, AngleUnit.DEGREES);
            flywheel1.setVelocity(650/28, AngleUnit.DEGREES);
            telemetryM.addData("Target Speed", 650);

        }
        else if (gamepad1.dpad_up){
            flywheel1.setVelocity(800/28, AngleUnit.DEGREES);
            flywheel1.setVelocity(800/28, AngleUnit.DEGREES);
            telemetryM.addData("Target Speed", 800);

        }
        else if (gamepad1.dpad_left){
            flywheel1.setVelocity(1000/28, AngleUnit.DEGREES);
            flywheel1.setVelocity(1000/28, AngleUnit.DEGREES);
            telemetryM.addData("Target Speed", 1000);

        }

        else if (gamepad1.left_trigger > 0.1){
            flywheel1.setPower(gamepad1.left_trigger);
            flywheel2.setPower(gamepad1.left_trigger);

        }


        else{
            flywheel1.setPower(0);
            flywheel2.setPower(0);
        }

        telemetryM.addData("Flywheel 1", flywheel1.getVelocity());
        telemetryM.addData("Flywheel 2", flywheel2.getVelocity());
        telemetryM.update();

    }
}