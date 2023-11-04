package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class TheJettCode extends LinearOpMode {
    private Servo LiftLeft;
    private Servo Hopper;
    double LiftHeight;
    boolean LiftMax;

    private void ServoNo() {
    }

    @Override
    public void runOpMode() throws InterruptedException {

        LiftLeft = hardwareMap.get(Servo.class, "LiftLeft");
        Hopper = hardwareMap.get(Servo.class, "Hopper");

        LiftLeft.getController().pwmEnable();
        Hopper.getController().pwmEnable();

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.y) {
                LiftHeight = 0.1;
            } else if (gamepad1.x) {
                LiftHeight = 0.5;
            } else {
                LiftHeight = 0;
            }


            LiftLeft.setPosition(LiftHeight);
            Hopper.setPosition(LiftHeight);
            telemetry.addData("LiftLeft", LiftLeft.getPosition());
            telemetry.addData("Hopper", Hopper.getPosition());
            telemetry.addData("Lift non var", LiftHeight);
            ServoNo();
            telemetry.update();
            sleep(100);
        }

    }


}