package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp
public class DistanceSensTest extends LinearOpMode {

    public static final double LEFT_FINGER_GRIP = 0.72;
    public static final double RIGHT_FINGER_GRIP = .27;
    Servo xyz;
    double DistanceDetection = 10;
    private Servo leftFinger;
    private Servo rightFinger;
    private DistanceSensor distanceL;
    private DistanceSensor distanceR;
    private void function() {
    }
    @Override
    public void runOpMode() throws InterruptedException {

        distanceL = hardwareMap.get(DistanceSensor.class, "DistanceL");
        distanceR = hardwareMap.get(DistanceSensor.class, "DistanceR");
        leftFinger = hardwareMap.get(Servo.class, "leftFinger");
        rightFinger = hardwareMap.get(Servo.class, "rightFinger");





        telemetry.update();
        waitForStart();

        while(opModeIsActive()) {
            if (distanceL.getDistance(DistanceUnit.CM) < DistanceDetection) {
                leftFinger.setPosition(LEFT_FINGER_GRIP);
            }

            if (distanceR.getDistance(DistanceUnit.CM) < DistanceDetection) {
                rightFinger.setPosition(RIGHT_FINGER_GRIP);
            }

            telemetry.update();
        }
    }
}
