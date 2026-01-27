package org.firstinspires.ftc.teamcode.pedroPathing.Tests;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Disabled
@TeleOp(name = "Intake Test", group = "Test")
public class OneMotorOneServo extends OpMode {
    private DcMotorEx intake;
    private ServoImplEx feed;


    @Override
    public void init() {
        intake = (DcMotorEx) hardwareMap.dcMotor.get("intake");
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intake.setPower(0);


        feed = hardwareMap.get(ServoImplEx.class, "feed");
        feed.setDirection(ServoImplEx.Direction.FORWARD);

    }

    @Override
    public void loop() {
        if (gamepad1.right_trigger > 0.1){
            intake.setPower(gamepad1.right_trigger);
        }
        else if (gamepad1.left_trigger > 0.1){
            intake.setPower(-gamepad1.left_trigger);
        }
        else{
            intake.setPower(0);
        }

        if (gamepad1.right_bumper){
            feed.setPosition(0.25);
        }
        else if (gamepad1.left_bumper){
            feed.setPosition(0);
        }

    }
}
