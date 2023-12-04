package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

//@TeleOp
public class McTuner2000 extends LinearOpMode {
    Servo wrist;
    Servo hopper;
    Servo shoulder;

    private void wristTuner() {
        if (gamepad1.y) {
            wrist.setPosition(wrist.getPosition() + 0.02);
        }
        else if (gamepad1.a) {
            wrist.setPosition(wrist.getPosition() - 0.02);
        }
        telemetry.addData("Wrist Position", wrist.getPosition());
    }
    private void hopperTuner() {
        if (gamepad1.x) {
            hopper.setPosition(hopper.getPosition() + 0.02);
        }
        else if (gamepad1.b) {
            hopper.setPosition(hopper.getPosition() - 0.02);
        }
        telemetry.addData("Hopper Position", hopper.getPosition());
    }
    @Override
    public void runOpMode() throws InterruptedException {

        wrist = hardwareMap.get(Servo.class, "wrist");
        hopper = hardwareMap.get(Servo.class, "hopper");

        wrist.setDirection(Servo.Direction.REVERSE);

        telemetry.addData("Wrist Position", wrist.getPosition());
        telemetry.addData("Hopper Position", hopper.getPosition());

        telemetry.update();
        waitForStart();
        //this is a coment to mAKE git update

        while(opModeIsActive()) {
            wristTuner();
            hopperTuner();
            telemetry.update();
        }
    }
}
