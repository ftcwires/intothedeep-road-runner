package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.vision.apriltag.AprilTagCanvasAnnotator;

//@TeleOp
public class McTuner3000 extends LinearOpMode {
    //TODO: Step 1, Replace all "wrist","hopper", etc with your servos
    Servo wrist;
    Servo hopper;
    Servo shoulder;
    Servo leftLift;
    Servo rightLift;

    DcMotor frontIntake;
    DcMotor rearIntake;

    double speedAmount;
    //TODO: Add offset if needed w/ eg double lift
    double LiftLeftOffset = -.05;

    //TODO: Step 3, set all of your servo names below
    enum ServoTypes{
        SHOULDER,
        HOPPER,
        WRIST,
        LIFT;
    }
    ServoTypes which;
    //TODO: Step 4, replace all names of Servos with yours, and replace all capitals with what you set them to from step 3

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
                leftLift.setPosition((rightLift.getPosition() +  LiftLeftOffset) - speedAmount);
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
                leftLift.setPosition((rightLift.getPosition() + LiftLeftOffset) + speedAmount);
                rightLift.setPosition(rightLift.getPosition() + speedAmount);
            }
        }
    }
    //TODO: Step 5, replace all of your Servo functions below
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
            speedAmount = 0.005;
        }
        else if (gamepad1.dpad_right) {
            speedAmount = 0.01;
        }
        else if (gamepad1.dpad_up) {
            speedAmount = 0.03;
        }
        telemetry.addData("Speed = ", speedAmount);
    }

    //TODO: Step 7, your done! This was written by Goober on 11/5/23 slouching in a chair at 10:35 in the morning.
    // And as I'm writing this I wonder if anybody will actually use this. Problably not,
    // but idk what to do while I wait for the robot to be ready for calibration. It's now 10:36 am.
    // I wonder if future me will use this and or remember doing this. Goober out.

    //TODO: Step 6 replace all of the xyz.getPosition()0; with your servos and replace "xyz" with what that servo is
    private void whatServoAt() {
        telemetry.addData("Shoulder = ", shoulder.getPosition());
        telemetry.addData("Hopper = ", hopper.getPosition());
        telemetry.addData("Wrist = ", wrist.getPosition());
        telemetry.addData("Lift Left = ",leftLift.getPosition());
        telemetry.addData("Lift Right = ",rightLift.getPosition());
    }
    private void slurp() {
        if (gamepad1.right_trigger > 0.5) {
            frontIntake.setPower(0.8);
            rearIntake.setPower(1);
            //leftLift.setPosition(.49);
            //rightLift.setPosition(.45);
            telemetry.addData("Intake", "in");
        } else if (gamepad1.left_trigger > 0.5) {
            frontIntake.setPower(-0.8);
            rearIntake.setPower(-1);

            telemetry.addData("Intake", "out");
        } else {
            telemetry.addData("Intake", "stopped");
            frontIntake.setPower(0);
            rearIntake.setPower(0);
        }
    }
    @Override
    public void runOpMode() throws InterruptedException {
        which = ServoTypes.SHOULDER;
        speedAmount = 0.01;
        //this is a coment to mAKE git update

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

        wrist.setPosition(0.265);
        shoulder.setPosition(0.455);
        hopper.setPosition(0.02);
        leftLift.setPosition(0.42 + LiftLeftOffset);
        rightLift.setPosition(0.42);


       // this is a comment
        frontIntake = hardwareMap.get(DcMotor.class, "frontIntake");
        rearIntake = hardwareMap.get(DcMotor.class, "rearIntake");

        frontIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        rearIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        frontIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rearIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rearIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        telemetry.update();
        waitForStart();

        while(opModeIsActive()) {
            setServo();
            slurp();
            masterTuner();
            setSpeed();
            whatServoAt();
            telemetry.addData("Selected", which.toString());
            telemetry.addLine("Y = Shoulder - X = Hopper - B = Wrist A = Lift");
            telemetry.update();
            sleep(100);
        }
    }
}
