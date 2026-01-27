package org.firstinspires.ftc.teamcode.pedroPathing.Tests;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.pedroPathing.HelpfulClasses.PositionPID;
import org.firstinspires.ftc.teamcode.pedroPathing.HelpfulClasses.Turret_GameSpecific;
import org.firstinspires.ftc.teamcode.pedroPathing.PID;


@TeleOp
public class Turret_PID_Tuning extends OpMode {
    private ServoImplEx hardStopL, hardStopR, feed;
    Turret_GameSpecific game = new Turret_GameSpecific();

    private TelemetryManager telemetryM;

    public double startingTicks = 0;
    private double targetDeg = 0;
    double power = 0;
    double error = 0;

    private PositionPID turretPID =
            new PositionPID(PID.autoaimP, PID.autoaimI, PID.autoaimD);

    @Override
    public void init() {
        game.init(hardwareMap);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

    }

    @Override
    public void start() {
        game.Start();
        game.setBlue();
    }

    @Override
    public void loop() {


//        LLResult result = game.limelight.getLatestResult();
//        if (result != null && result.isValid()){
//            telemetry.addLine("ITS LOOKING AT SOMETHING");
//            telemetry.addLine("The Distance Is... " + result.getBotposeAvgDist());
//            telemetry.update();
//        }






//        if (gamepad1.right_trigger > 0.1){
//           game.intake.setPower(gamepad1.right_trigger);
//        }
//        else if (gamepad2.left_trigger > 0.1){
//            game.intake.setPower(gamepad1.left_trigger);
//        }



//        if (gamepad2.x){
//            feed.setPosition(SERVO_POSITIONS.feedStart);
//        }
//        if (gamepad2.y){
//            feed.setPosition(SERVO_POSITIONS.feedEnd);
//        }
//        if (gamepad2.right_trigger > 0.1)
//            hardStopL.setPosition(SERVO_POSITIONS.gateStartL);
//        if (gamepad2.right_bumper){
//            hardStopL.setPosition(SERVO_POSITIONS.gateEndL);
//        }
//        if (gamepad2.left_bumper){
//            hardStopR.setPosition(SERVO_POSITIONS.gateEndR);
//        }
//        if (gamepad2.left_trigger > 0.1){
//            hardStopR.setPosition(SERVO_POSITIONS.gateStartR);
//        }


        // Button-based target selection
        if (gamepad1.a) targetDeg = 5;
        if (gamepad1.b) targetDeg = 20;
        if (gamepad1.y) targetDeg = -5;
        if (gamepad1.x) targetDeg = -20;

        if (gamepad1.dpad_left) targetDeg = -1;
        if (gamepad1.dpad_up) targetDeg = 0;
        if (gamepad1.dpad_right) targetDeg = -1;

        if (gamepad1.right_trigger>0){
            targetDeg = game.turretTickstoDegrees();
        }

        power = turretPID.update(targetDeg,game.turretTickstoDegrees());

        game.tRotate.setPower(power);


//
//
        telemetryM.addData("Error", turretPID.error);
        telemetryM.addData("Target (deg)", targetDeg);
        telemetryM.addData("Current (deg)", game.turretTickstoDegrees());
        telemetryM.addData("Motor Power", power);
//
        telemetryM.addData("Turret Degrees", game.turretTickstoDegrees());
        telemetryM.addData("Tx", game.limelight.getLatestResult().getTx());
        telemetryM.addData("Encoder Ticks", game.tRotate.getCurrentPosition());
        telemetryM.update();
    }
}
