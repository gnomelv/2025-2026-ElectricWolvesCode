package org.firstinspires.ftc.teamcode.pedroPathing.Tests;


import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.HelpfulClasses.Turret_GameSpecific;

@TeleOp(name= "LL TEST", group= "Test")
public class LL_Test extends OpMode {

    TelemetryManager telemetryM;
    Turret_GameSpecific game = new Turret_GameSpecific();
    Pose robotPose = new Pose(0,0,0);


    @Override
    public void init() {
        game.init(hardwareMap);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }


    @Override
    public void loop() {
        if (gamepad1.a){
            robotPose = game.limelightGetPose();
        }




    }
}
