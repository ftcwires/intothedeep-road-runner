package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class TheJettCode extends LinearOpMode {
    private CRServo Tape;
    double TapePower;
    private void tapetroll() {
        if (gamepad1.a) {
            TapePower = 0.7;
            Tape.setPower(TapePower);
            sleep(570);
            TapePower = 0;
            Tape.setPower(TapePower);
        }

    }

    private void taper() {
        if (gamepad1.y) {
            TapePower = -0.7;
        } else if (gamepad1.x) {
            TapePower = 0.7;
        } else {
            TapePower = 0;
        }
        Tape.setPower(TapePower);
        telemetry.addData("TAPE", Tape.getPower());
        telemetry.addData("TAPE non var", TapePower);
    }
    @Override
    public void runOpMode() throws InterruptedException {


        Tape = hardwareMap.get(CRServo.class, "Tape");
        Tape.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        while (opModeIsActive()) {
            taper();
            tapetroll();
            telemetry.update();
        }

    }



}