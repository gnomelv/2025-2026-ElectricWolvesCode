package org.firstinspires.ftc.teamcode.pedroPathing.Tests;


import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.HelpfulClasses.Turret_GameSpecific;
import org.firstinspires.ftc.teamcode.pedroPathing.SERVO_POSITIONS;

@TeleOp(name = "Intake Tuning")
public class IntakeTuning extends OpMode {
    Turret_GameSpecific game = new Turret_GameSpecific();
    double targetSpeed = 0;
    double intakeSpeed = 0;
    private LynxModule hub;
    private TelemetryManager telemetryM;

    @Override
    public void init() {
        game.init(hardwareMap);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        hub = hardwareMap.getAll(LynxModule.class).get(0);
    }

    @Override
    public void start() {
        game.Start();
        game.hardStopR.setPosition(SERVO_POSITIONS.gateEndR);
        game.hardStopL.setPosition(SERVO_POSITIONS.gateEndL);
    }

    @Override
    public void loop() {
        if (gamepad1.a){
            targetSpeed = 2000;
        }
        else if (gamepad1.b){
            targetSpeed = 1900;
        }
        else if (gamepad1.y){
            targetSpeed = 1800;
        }
        else if (gamepad1.x){
           targetSpeed = 1700;
        }
        else if (gamepad1.right_bumper) {targetSpeed = 0;}




//        if (gamepad1.dpad_down){
//            intakeSpeed = 1800;
//        }
//        else if (gamepad1.dpad_right){
//            intakeSpeed = 1600;
//        }
//        else if (gamepad1.dpad_up){
//            intakeSpeed = 1400;
//        }
//        else if (gamepad1.dpad_left){
//            intakeSpeed = 1200;
////        }
//        else if (gamepad1.left_bumper) {intakeSpeed = 0;}


        //game.intake.setPower(game.intake_PID.update(intakeSpeed, game.intake.getVelocity()));

        double input1 = game.intake_PID.update(targetSpeed, game.flywheel1.getVelocity(), hub.getInputVoltage(VoltageUnit.VOLTS));

        if (gamepad1.right_trigger> 0.1){
            input1 = 1;
        }

        if (targetSpeed == 0 && !(gamepad1.right_trigger > 0.1)){
            input1 =0;
        }


        game.intake.setPower(input1);



        telemetryM.addData("Intake  Input", input1);
        telemetryM.addData("Intake Speed", game.intake.getVelocity());
        telemetryM.addData("Target Speed", targetSpeed);
        telemetryM.addData("Intake Current", game.intake.getCurrent(CurrentUnit.AMPS));


//        telemetryM.addData("Intake Speed", game.intake.getVelocity());
//        telemetryM.addData("Intake Target", intakeSpeed);

        telemetryM.update();




    }
}
