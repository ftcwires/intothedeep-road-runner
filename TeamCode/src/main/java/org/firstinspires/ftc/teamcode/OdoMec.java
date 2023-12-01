package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

//@TeleOp(name="TeleMec")

public class OdoMec extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    //Motors
    //Drivetrain
    private DcMotor rightFront; //front right 0
    private DcMotor leftFront; //front left 2
    private DcMotor rightBack; //rear right 1
    private DcMotor leftBack; //rear left 3



    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");

        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);

        //Set motor modes
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        telemetry.addData("Status", "OdoMec2 is ready to run!");
        telemetry.update();

        //Wait for press play
        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            //Drivetrain
            double forward = gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;
            double turn = -gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(turn), 1);

            double rightFrontPower = (forward - strafe - turn) / denominator;
            double leftFrontPower = (forward + strafe + turn) / denominator;
            double rightBackPower = (forward + strafe - turn) / denominator;
            double leftBackPower = (forward - strafe + turn) / denominator;

            if (gamepad1.left_bumper) {
                rightFrontPower = Range.clip(rightFrontPower, -0.4, 0.4);
                leftFrontPower = Range.clip(leftFrontPower, -0.4, 0.4);
                rightBackPower = Range.clip(rightBackPower, -0.4, 0.4);
                leftBackPower = Range.clip(leftBackPower, -0.4, 0.4);
            } else {
                rightFrontPower = Range.clip(rightFrontPower, -0.8, 0.8);
                leftFrontPower = Range.clip(leftFrontPower, -0.8, 0.8);
                rightBackPower = Range.clip(rightBackPower, -0.8, 0.8);
                leftBackPower = Range.clip(leftBackPower, -0.8, 0.8);
            }


            rightFront.setPower(rightFrontPower);
            leftFront.setPower(leftFrontPower);
            rightBack.setPower(rightBackPower);
            leftBack.setPower(leftBackPower);

            telemetry.addData("Status", "Run " + runtime.toString());
            telemetry.addData("Motors", "forward (%.2f), strafe (%.2f),turn (%.2f)", forward, strafe, turn);
            telemetry.update();


            // run until the end of
        }
    }
}
