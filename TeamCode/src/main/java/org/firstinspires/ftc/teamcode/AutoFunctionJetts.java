package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

//@Autonomous
public class AutoFunctionJetts extends LinearOpMode {
    private DcMotor rightFront; //front right 0
    private DcMotor leftFront; //front left 2
    private DcMotor rightBack; //rear right 1
    private DcMotor leftBack;
    DcMotor frontIntake;
    DcMotor rearIntake;
    private Servo shoulder;
    private Servo wrist;
    private Servo hopper;

public void FirstTest() {
    rightFront.setPower(0.2);
    rightBack.setPower(0.2);
    leftBack.setPower(0.2);
    leftFront.setPower(0.2);
    sleep(300);
    rightFront.setPower(-0.25);
    rightBack.setPower(-0.25);
    leftBack.setPower(0.25);
    leftFront.setPower(0.25);
    sleep(500);
    rightFront.setPower(0.2);
    rightBack.setPower(0.2);
    leftBack.setPower(0.2);
    leftFront.setPower(0.2);
    sleep(300);
    frontIntake.setPower(-0.3);
    rearIntake.setPower(-0.3);


}

    @Override
    public void runOpMode() throws InterruptedException {
        //Wheel init
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        //Intake init
        frontIntake = hardwareMap.get(DcMotor.class, "frontIntake");
        rearIntake = hardwareMap.get(DcMotor.class, "rearIntake");
        //Holder init
        wrist = hardwareMap.get(Servo.class, "wrist");
        hopper = hardwareMap.get(Servo.class, "hopper");
        //Arm init
        shoulder = hardwareMap.get(Servo.class, "shoulder");
        //Wheel Set
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        //Intake Set
        frontIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        rearIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        frontIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rearIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rearIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Holder Set
        shoulder.setDirection(Servo.Direction.REVERSE);
        wrist.setDirection(Servo.Direction.REVERSE);



        FirstTest();

    }
}
