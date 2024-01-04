package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

//@TeleOp
public class WorkingTeleOpFramework extends LinearOpMode {
    Servo xyz;

    private void function() {
    }
    @Override
    public void runOpMode() throws InterruptedException {

        xyz = hardwareMap.get(Servo.class, "xyz");

        telemetry.update();
        waitForStart();

        while(opModeIsActive()) {

            telemetry.update();
        }
    }
}
