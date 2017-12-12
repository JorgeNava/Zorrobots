package org.firstinspires.ftc.teamcode.RelicRecovery.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Jorge on 14/11/2017.
 */
@TeleOp(name ="TeleOpRR_Proto",group = "TeleOp")
public class TeleOpRR_Proto extends OpMode {

    //WHEELS
    public DcMotor Left_motors = null;
    public DcMotor Right_motors = null;
    public DcMotor Lifter = null;

    public Servo RightJServo = null;
    public Servo LeftJServo = null;
    public Servo LeftCServo = null;

    public Servo RightCServo = null;


    @Override
    public void init() {
        Left_motors = hardwareMap.dcMotor.get("Left_motors");
        Right_motors = hardwareMap.dcMotor.get("Right_motors");
        Lifter = hardwareMap.dcMotor.get("Lifter");

        Right_motors.setDirection(DcMotor.Direction.REVERSE);
     //   LeftJServo.setDirection(Servo.Direction.REVERSE);

        RightJServo = hardwareMap.servo.get("RightJServo");
        LeftJServo = hardwareMap.servo.get("LeftJServo");
        LeftCServo = hardwareMap.servo.get("LeftCServo");
        RightCServo = hardwareMap.servo.get("RightCServo");
    }

    @Override
    public void loop() {
        //WHEELS
        Left_motors.setPower(gamepad1.left_stick_y);
        Right_motors.setPower(gamepad1.right_stick_y);

        //JEWEL
        if (gamepad1.right_bumper) {
            RightJServo.setPosition(.7);
        } else if (gamepad1.left_bumper) {
            LeftJServo.setPosition(0);
        } else {
            RightJServo.setPosition(0);
            LeftJServo.setPosition(.7);
        }

        //CUBE
        if (gamepad1.x){
            RightCServo.setPosition(0);
            LeftCServo.setPosition(.7);
        }
        else{
            RightCServo.setPosition(.5);
            LeftCServo.setPosition(0);
        }

        if(gamepad1.a){
            Lifter.setPower(1);
        }
        else if(gamepad1.y){
            Lifter.setPower(-.3);
        }
        else{
            Lifter.setPower(0);
        }
    }
}
