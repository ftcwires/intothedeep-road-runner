package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class StopMotionTester {

    static public void Dominator(HardwareMap hardwareMap) {
        Servo wrist;
        Servo hopper;
        Servo shoulder;
        Servo leftLift;
        Servo rightLift;

        wrist = hardwareMap.get(Servo.class, "wrist");
        hopper = hardwareMap.get(Servo.class, "hopper");
        shoulder = hardwareMap.get(Servo.class, "shoulder");
        rightLift = hardwareMap.get(Servo.class, "rightLift");
        leftLift = hardwareMap.get(Servo.class, "leftLift");

        shoulder.setDirection(Servo.Direction.REVERSE);
        wrist.setDirection(Servo.Direction.REVERSE);

        wrist.setPosition(0.51);
        shoulder.setPosition(0.44);
        hopper.setPosition(0);
        leftLift.setPosition(0.42);
        rightLift.setPosition(0.42);

        sleep(2000);

        wrist.setPosition(0.74);
        shoulder.setPosition(0.7);
        hopper.setPosition(0);

        sleep(2000);

        wrist.setPosition(0.73);
        shoulder.setPosition(0.75);
        hopper.setPosition(0.16);

        sleep(2000);



    }
}
