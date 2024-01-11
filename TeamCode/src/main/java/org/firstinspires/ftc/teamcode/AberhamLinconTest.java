package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class AberhamLinconTest extends LinearOpMode {
    private ColorSensor color;

    private void function() {
    }
    @Override
    public void runOpMode() throws InterruptedException {

        color = hardwareMap.get(ColorSensor.class, "color");

        telemetry.update();
        waitForStart();
        color.enableLed(true);
        color.green();

        while(opModeIsActive()) {


            telemetry.update();
        }
    }
}
