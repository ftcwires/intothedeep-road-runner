package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp
public class DistanceSensTest extends LinearOpMode {
    double DistanceDetection = 10;
    private Servo leftFinger;
    private Servo rightFinger;
    private DistanceSensor distanceL;
    private DistanceSensor distanceR;
    @Override
    public void runOpMode() throws InterruptedException {
    //Hardware Mapping ↓
        distanceL = hardwareMap.get(DistanceSensor.class, "DistanceL");
        distanceR = hardwareMap.get(DistanceSensor.class, "DistanceR");
        leftFinger = hardwareMap.get(Servo.class, "lFinger");
        rightFinger = hardwareMap.get(Servo.class, "rFinger");

        telemetry.update();
        waitForStart();
    //CODE ↓
        while(opModeIsActive()) {
            if (distanceL.getDistance(DistanceUnit.CM) < DistanceDetection) {
                leftFinger.setPosition(Evolution.LEFT_FINGER_GRIP);
            }

            if (distanceR.getDistance(DistanceUnit.CM) < DistanceDetection) {
                rightFinger.setPosition(Evolution.RIGHT_FINGER_GRIP);
            }

            telemetry.update();
        }
    }
}
