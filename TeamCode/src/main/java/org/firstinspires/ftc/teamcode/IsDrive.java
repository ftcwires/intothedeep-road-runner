package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class IsDrive {
    DcMotor BackLeft;
    DcMotor BackRight;
    DcMotor FrontLeft;
    DcMotor FrontRight;

    public IsDrive(HardwareMap hardwareMap) {

        BackLeft = hardwareMap.get(DcMotor.class, "leftBack");
        BackRight = hardwareMap.get(DcMotor.class, "rightBack");
        FrontLeft = hardwareMap.get(DcMotor.class, "leftFront");
        FrontRight = hardwareMap.get(DcMotor.class, "rightFront");


        // init servo hardware
        // init drive hardware and variables
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        //BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void is_drive_code(Gamepad gamepad1, Telemetry telemetry) {
        double forward = gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(forward)+Math.abs(strafe)+Math.abs(turn), 1);

        double rightFrontPower = (forward - strafe - turn) / denominator;
        double leftFrontPower = (forward + strafe + turn) / denominator;
        double rightRearPower = (forward + strafe - turn) / denominator;
        double leftRearPower = (forward - strafe + turn) / denominator;

        if (gamepad1.left_bumper) {
            rightFrontPower = Range.clip(rightFrontPower, -0.4, 0.4);
            leftFrontPower = Range.clip(leftFrontPower, -0.4, 0.4);
            rightRearPower = Range.clip(rightRearPower, -0.4, 0.4);
            leftRearPower = Range.clip(leftRearPower, -0.4, 0.4);
        } else {
            rightFrontPower = Range.clip(rightFrontPower, -0.8, 0.8);
            leftFrontPower = Range.clip(leftFrontPower, -0.8, 0.8);
            rightRearPower = Range.clip(rightRearPower, -0.8, 0.8);
            leftRearPower = Range.clip(leftRearPower, -0.8, 0.8);
        }


        FrontRight.setPower(rightFrontPower);
        FrontLeft.setPower(leftFrontPower);
        BackRight.setPower(rightRearPower);
        BackLeft.setPower(leftRearPower);
    }
}
