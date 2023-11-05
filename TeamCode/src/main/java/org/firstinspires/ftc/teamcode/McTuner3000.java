package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.vision.apriltag.AprilTagCanvasAnnotator;

@TeleOp
public class McTuner3000 extends LinearOpMode {
    //TODO: Step 1, Replace all "wrist","hopper", etc with your servos
    Servo wrist;
    Servo hopper;
    Servo shoulder;
    Servo leftLift;
    Servo rightLift;

    double speedAmount;

    enum ServoTypes{
        SHOULDER,
        HOPPER,
        WRIST,
        LIFT;
    }
    ServoTypes which;

    private void masterTuner() {
        if (gamepad1.left_bumper) {
            if (which == ServoTypes.SHOULDER) {
                shoulder.setPosition(shoulder.getPosition() - speedAmount);
            }
            else if (which == ServoTypes.HOPPER) {
                hopper.setPosition(hopper.getPosition() - speedAmount);
            }
            else if (which == ServoTypes.WRIST) {
                wrist.setPosition(wrist.getPosition() - speedAmount);
            }
            else if (which == ServoTypes.LIFT) {
                leftLift.setPosition(leftLift.getPosition() - speedAmount);
                rightLift.setPosition(rightLift.getPosition() - speedAmount);
            }
        }
        else if (gamepad1.right_bumper) {
            if (which == ServoTypes.SHOULDER) {
                shoulder.setPosition(shoulder.getPosition() + speedAmount);
            }
            else if (which == ServoTypes.HOPPER) {
                hopper.setPosition(hopper.getPosition() + speedAmount);
            }
            else if (which == ServoTypes.WRIST) {
                wrist.setPosition(wrist.getPosition() + speedAmount);
            }
            else if (which == ServoTypes.LIFT) {
                leftLift.setPosition(leftLift.getPosition() + speedAmount);
                rightLift.setPosition(rightLift.getPosition() + speedAmount);
            }
        }
    }
    private void setServo() {
        if (gamepad1.y) {
            which = ServoTypes.SHOULDER;
        }
        else if (gamepad1.x) {
            which = ServoTypes.HOPPER;
        }
        else if (gamepad1.b) {
            which = ServoTypes.WRIST;
        }
        else if (gamepad1.a) {
            which = ServoTypes.LIFT;
        }
        telemetry.addData("Selected Servo = ", which.toString());
    }
    private void setSpeed() {
        if (gamepad1.dpad_down) {
            speedAmount = 0;
        }
        else if (gamepad1.dpad_left) {
            speedAmount = 0.01;
        }
        else if (gamepad1.dpad_right) {
            speedAmount = 0.02;
        }
        else if (gamepad1.dpad_up) {
            speedAmount = 0.05;
        }
        telemetry.addData("Speed = ", speedAmount);
    }
    private void whatServoAt() {
        telemetry.addData("Shoulder = ", shoulder.getPosition());
        telemetry.addData("Hopper = ", hopper.getPosition());
        telemetry.addData("Wrist = ", wrist.getPosition());
        telemetry.addData("Lift Left = ",leftLift.getPosition());
        telemetry.addData("Lift Right = ",rightLift.getPosition());
    }
    @Override
    public void runOpMode() throws InterruptedException {
        which = ServoTypes.SHOULDER;
        speedAmount = 0.01;

        //TODO: Step 2, Replace the device names with your 4 (or more if you use two servos for one task)
        wrist = hardwareMap.get(Servo.class, "wrist");
        hopper = hardwareMap.get(Servo.class, "hopper");
        shoulder = hardwareMap.get(Servo.class, "shoulder");
        rightLift = hardwareMap.get(Servo.class, "rightLift");
        leftLift = hardwareMap.get(Servo.class, "leftLift");

        shoulder.setDirection(Servo.Direction.REVERSE);
        wrist.setDirection(Servo.Direction.REVERSE);

        telemetry.addData("Wrist Position", wrist.getPosition());
        telemetry.addData("Hopper Position", hopper.getPosition());

        telemetry.update();
        waitForStart();

        while(opModeIsActive()) {
            setServo();
            masterTuner();
            setSpeed();
            whatServoAt();
            telemetry.addData("Selected", which.toString());
            telemetry.addLine("Y = Shoulder - X = Hopper - B = Wrist A = Lift");
            telemetry.update();
        }
    }
}
