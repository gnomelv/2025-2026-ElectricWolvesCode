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

@TeleOp(name = "Flywheel Tuning")
public class FlywheelTuning extends OpMode {
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
            targetSpeed = 2100;
        }
        else if (gamepad1.y){
            targetSpeed = 2200;
        }
        else if (gamepad1.x){
           targetSpeed = 1900;
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

        double input1 = game.fly1_PID.update(targetSpeed, game.flywheel1.getVelocity(), hub.getInputVoltage(VoltageUnit.VOLTS));

        if (gamepad1.right_trigger> 0.1){
            input1 = 1;
        }

        if (targetSpeed == 0 && !(gamepad1.right_trigger > 0.1)){
            input1 =0;
        }


        game.flywheel1.setPower(input1);
        game.flywheel2.setPower(input1);


        telemetryM.addData("1 Over Current",game.flywheel1.isOverCurrent());
        telemetryM.addData("2 Over Current",game.flywheel2.isOverCurrent());
        telemetryM.addData("Fly 1  Input", input1);
        telemetryM.addData("Detected Power 1", game.flywheel1.getPower());
        telemetryM.addData("Detected Power 2", game.flywheel2.getPower());
        telemetryM.addData("Fly1 Speed", game.flywheel1.getVelocity());
        telemetryM.addData("Fly2 Speed", game.flywheel2.getVelocity());
        telemetryM.addData("Target Speed", targetSpeed);
        telemetryM.addData("Fly 2 Current", game.flywheel2.getCurrent(CurrentUnit.AMPS));
        telemetryM.addData("Fly 1 Current", game.flywheel1.getCurrent(CurrentUnit.AMPS));

//        telemetryM.addData("Intake Speed", game.intake.getVelocity());
//        telemetryM.addData("Intake Target", intakeSpeed);

        telemetryM.update();




    }
}
