package org.firstinspires.ftc.teamcode.VelocityVortex.Old;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Jorge on 16/12/2016.
 */

   // @TeleOp(name ="TeleOp2016_3",group = "TeleOp")
    public class TeleOp2016_3 extends LinearOpMode
    {
        public DcMotor motorLeftF;
        public DcMotor motorRightF;
        public DcMotor motorLeftB;
        public DcMotor motorRightB;
        public DcMotor LauncherR;
        public DcMotor LauncherL;
        public DcMotor aspas;
        public DcMotor banda;
        public Servo ServoR;
        public Servo ServoL;
        public ColorSensor LineSensor;
        public ColorSensor BeaconSensor;
        public OpticalDistanceSensor WallSensor;


        @Override
        public void runOpMode() throws InterruptedException {

            motorLeftF = hardwareMap.dcMotor.get("motorLeftF");
            motorLeftB = hardwareMap.dcMotor.get("motorLeftB");
            motorRightF = hardwareMap.dcMotor.get("motorRightF");
            motorRightB = hardwareMap.dcMotor.get("motorRightB");
            LauncherL = hardwareMap.dcMotor.get("LauncherL");
            LauncherR = hardwareMap.dcMotor.get("LauncherR");
            aspas = hardwareMap.dcMotor.get("aspas");
            banda = hardwareMap.dcMotor.get("banda");
            LineSensor = hardwareMap.colorSensor.get("LineSensor");
            BeaconSensor = hardwareMap.colorSensor.get("BeaconSensor");
            WallSensor = hardwareMap.opticalDistanceSensor.get("WallSensor");
            ServoL = hardwareMap.servo.get("ServoL");
            ServoR = hardwareMap.servo.get("ServoR");
            motorLeftF.setDirection(DcMotor.Direction.REVERSE);
            motorLeftB.setDirection(DcMotor.Direction.REVERSE);
            LauncherL.setDirection(DcMotor.Direction.REVERSE);
            LineSensor.enableLed(false);
            BeaconSensor.enableLed(false);

            waitForStart();

            while (opModeIsActive())
            {
                //WHEELS
                motorLeftB.setPower(gamepad1.left_stick_y);
                motorLeftF.setPower(gamepad1.left_stick_y);
                motorRightB.setPower(gamepad1.right_stick_y);
                motorRightF.setPower(gamepad1.right_stick_y);

                //BLADES
                aspas.setPower(gamepad2.left_stick_y);

                //INVERT GAMEPAD CONTROLERS
                if (gamepad1.dpad_up && gamepad1.y) {
                    motorRightF.setDirection(DcMotor.Direction.FORWARD);
                    motorRightB.setDirection(DcMotor.Direction.FORWARD);
                    motorLeftF.setDirection(DcMotor.Direction.REVERSE);
                    motorLeftF.setDirection(DcMotor.Direction.REVERSE);
                }
                if (gamepad1.dpad_left && gamepad1.x) {
                    motorRightF.setDirection(DcMotor.Direction.REVERSE);
                    motorRightB.setDirection(DcMotor.Direction.REVERSE);
                    motorLeftF.setDirection(DcMotor.Direction.FORWARD);
                    motorLeftF.setDirection(DcMotor.Direction.FORWARD);
                }

                //LAUNCHERS
                if (gamepad1.right_bumper){
                    LauncherL.setPower(1);
                    LauncherR.setPower(1);
                }
                else {
                    LauncherL.setPower(0);
                    LauncherR.setPower(0);
                }

                //SERVOS
                if(gamepad2.right_bumper){
                    ServoR.setPosition(0);
                }
                else {
                    ServoR.setPosition(1);
                }
                if (gamepad2.left_bumper){
                    ServoL.setPosition(1);
                }
                else {
                    ServoL.setPosition(0);
                }

                //RUBBER BAND
                if (gamepad2.x)
                {
                    banda.setPower(1);
                }
                else if  (gamepad2.y)
                {
                    banda.setPower(-1);
                }
                else
                {
                    banda.setPower(0);
                }


                idle();
            }
        }
    }


