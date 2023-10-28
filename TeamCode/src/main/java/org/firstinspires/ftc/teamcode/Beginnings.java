package org.firstinspires.ftc.teamcode;

// imports

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.android.AndroidTextToSpeech;

@TeleOp
public class Beginnings extends LinearOpMode {
    // Declare vars
    DcMotor BackLeft;
    DcMotor BackRight;
    DcMotor FrontLeft;
    DcMotor FrontRight;
    Servo LiftRight;
    Servo LiftLeft;

    // Servo prep

    double MinLiftHeight = 0.05;
    double MaxLiftHeight = 0.65;
    double LiftLeftOffset = -0.08;
    double LiftHeight;
    private AndroidTextToSpeech androidTextToSpeech;



    // Functions \/

    private void drive_code() {

        double Scale_Factor_of_Drive;

        if (gamepad1.right_bumper) {
            Scale_Factor_of_Drive = 1;
        } else {
            Scale_Factor_of_Drive = 0.55;
        }
        // drive with joysticks
        BackLeft.setPower(-0.8 * Scale_Factor_of_Drive * gamepad1.right_stick_x - 0.8 * Scale_Factor_of_Drive * (gamepad1.left_stick_x + gamepad1.left_stick_y));
        BackRight.setPower(0.8 * Scale_Factor_of_Drive * gamepad1.right_stick_x - -0.8 * Scale_Factor_of_Drive * (gamepad1.left_stick_x - gamepad1.left_stick_y));
        FrontLeft.setPower(-0.8 * Scale_Factor_of_Drive * gamepad1.right_stick_x - -0.8 * Scale_Factor_of_Drive * (gamepad1.left_stick_x - gamepad1.left_stick_y));
        FrontRight.setPower(-0.8 * Scale_Factor_of_Drive * gamepad1.right_stick_x - -0.8 * Scale_Factor_of_Drive * (gamepad1.left_stick_x + gamepad1.left_stick_y));
        telemetry.addData("FL Motor", FrontLeft.getPower());
        telemetry.addData("FR Motor", FrontRight.getPower());
        telemetry.addData("BL Motor", BackLeft.getPower());
        telemetry.addData("BR Motor", BackRight.getPower());

    }

    private void servo_shenanigans() {
       setLiftHeight(0.4);
       sleep(10000);
       setLiftHeight(0.05);
    }

    private void setLiftHeight(double inputLiftHeight) {
        if (inputLiftHeight < 0.05){
            inputLiftHeight = 0.05;
        }
        if (inputLiftHeight > 0.65){
            inputLiftHeight = 0.65;
        }
        LiftHeight = inputLiftHeight;
        LiftLeft.setPosition(LiftLeftOffset + LiftHeight);
        LiftRight.setPosition(LiftHeight);

    }


    @Override
    // NOT loop \/ - Or int of vars
    public void runOpMode() throws InterruptedException {

        androidTextToSpeech = new AndroidTextToSpeech();
        androidTextToSpeech.initialize();

        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        LiftRight = hardwareMap.get(Servo.class, "LiftRight");
        LiftLeft = hardwareMap.get(Servo.class, "LiftLeft");

        LiftRight.setDirection(Servo.Direction.REVERSE);


        // init servo hardware
        // init drive hardware and variables
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setDirection(DcMotorSimple.Direction.REVERSE);




        waitForStart();
        servo_shenanigans();
        // loop real
        while(opModeIsActive()){
            //drive_code();
            telemetry.update();
            sleep(100);
        }
    }
}
