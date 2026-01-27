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

@TeleOp(name = "Hood Calibration")
public class Hood_Calibration extends OpMode {
    Turret_GameSpecific game = new Turret_GameSpecific();
    private TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    private double hoodMin = 0;
    private double hoodMax = 0.52;
    private double hoodTarget = 0;
    private double flywheelTarget = 0;
    private LynxModule hub;

    double input2 = 0;
    double input1 = 0;

    @Override
    public void init() {
        game.init(hardwareMap);
        hub = hardwareMap.getAll(LynxModule.class).get(0);
    }

    @Override
    public void start(){
        game.tAngle.setPosition(hoodMin);
        game.hardStopL.setPosition(SERVO_POSITIONS.gateEndL);
        game.hardStopR.setPosition(SERVO_POSITIONS.gateEndR);
    }

    @Override
    public void loop() {
        if (Math.abs(gamepad1.left_stick_y) > 0.1){
            hoodTarget -= gamepad1.left_stick_y * 0.05;
        }
        if (Math.abs(gamepad1.right_stick_y) > 0.1){
            hoodTarget -= gamepad1.right_stick_y * 0.01;
        }
        if (hoodTarget > hoodMax) hoodTarget = hoodMax;
        if (hoodTarget < hoodMin) hoodTarget = hoodMin;

        if (Math.abs(hoodTarget - game.tAngle.getPosition()) > 0.01){
            game.tAngle.setPosition(hoodTarget);
        }

        if (gamepad1.a){flywheelTarget = 1900;}
        if (gamepad1.b){flywheelTarget = 1950;}
        if (gamepad1.x){flywheelTarget = 2000;}
        if (gamepad1.y){flywheelTarget = 2500;}
        if (gamepad1.dpad_up){flywheelTarget = 2100;}
        if (gamepad1.dpad_down){flywheelTarget = 2150;}
        if (gamepad1.dpad_left){flywheelTarget = 2200;}
        if (gamepad1.dpad_right){flywheelTarget = 2250;}
        if (gamepad1.right_bumper || gamepad1.left_bumper){flywheelTarget = 0;}

        //input1 = game.fly1_PID.update(flywheelTarget, game.flywheel1.getVelocity());
        if (gamepad2.right_trigger > 0.1){
            flywheelTarget += gamepad2.right_trigger * 5;
        }
        if (gamepad2.left_trigger > 0.1){
            flywheelTarget -= gamepad2.left_trigger * 5;
        }

        input2 = game.fly1_PID.update(flywheelTarget, game.flywheel1.getVelocity(), hub.getInputVoltage(VoltageUnit.VOLTS));

        if (flywheelTarget == 0){input2 = 0;}

        game.flywheel1.setPower(input2);
        game.flywheel2.setPower(input2);

        if (gamepad1.right_trigger > 0.1){
            game.intake.setPower(gamepad1.right_trigger);
        }
        else {game.intake.setPower(0);}




        telemetryM.addData("Hood Target", hoodTarget);
        telemetryM.addData("1 Over Current",game.flywheel1.isOverCurrent());
        telemetryM.addData("2 Over Current",game.flywheel2.isOverCurrent());
        //telemetryM.addData("Fly 1  Input", input1);
        telemetryM.addData("Fly 2  Input", input2);
        telemetryM.addData("Detected Power 1", game.flywheel1.getPower());
        telemetryM.addData("Detected Power 2", game.flywheel2.getPower());
        telemetryM.addData("Fly1 Speed", game.flywheel1.getVelocity());
        telemetryM.addData("Fly2 Speed", game.flywheel2.getVelocity());
        telemetryM.addData("Target Speed", flywheelTarget);
        telemetryM.addData("Fly 2 Current", game.flywheel2.getCurrent(CurrentUnit.AMPS));
        telemetryM.addData("Fly 1 Current", game.flywheel1.getCurrent(CurrentUnit.AMPS));
        telemetryM.update();


        telemetry.addLine("Use left trigger and right joysticks to adjust the hood");
        telemetry.addLine("Use buttons and the dpad to select different flywheel speeds");
        telemetry.addLine("The right and left buttons stop the flywheel");
        telemetry.addLine("Adjust what speeds different buttons do in the actual code");
        telemetry.addLine("Use PANELS to check that the flywheels are up to speed");
        telemetry.addLine("Manually feed a ball in and test what angles are needed to get the ball ");
        telemetry.addLine("in at different speeds from the point of the close zone");
        telemetry.addLine("NOTE WHAT ANGLES ARE NEEDED FOR EACH SPEED. MAKE A TABLE");
        telemetry.addLine("BE VERY SURE THAT THE BALL WILL SCORE FOR EACH DATA POINT");

    }
}
