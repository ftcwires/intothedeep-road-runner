package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class TheJettCode extends LinearOpMode {
    private Servo LiftLeft;
    double LiftHeight;
    private void ServoNo() {
        if (gamepad1.a) {
            LiftHeight = 0.7;
            LiftLeft.setPosition(LiftHeight);
            sleep(570);
            LiftHeight = 0;
            LiftLeft.setPosition(LiftHeight);
        }

    }

    private void ServoYes() {
        if (gamepad1.y) {
            LiftHeight = -0.7;
        } else if (gamepad1.x) {
            LiftHeight = 0.7;
        } else {
            LiftHeight = 0;
        }
        LiftLeft.setPosition(LiftHeight);
        telemetry.addData("TAPE", LiftLeft.getPosition());
        telemetry.addData("TAPE non var", LiftHeight);
    }
    @Override
    public void runOpMode() throws InterruptedException {


        LiftLeft = hardwareMap.get(Servo.class, "LiftLeft");
        waitForStart();
        while (opModeIsActive()) {
            ServoYes();
            ServoNo();
            telemetry.update();
        }

    }



}