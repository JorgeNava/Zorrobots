package org.firstinspires.ftc.teamcode.VelocityVortex.ColorSensor;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Jorge on 15/12/2016.
 */
//@Autonomous(name ="BEACONS")
public class BEACONS extends LinearOpMode {

    private DcMotor motorLeftF;
    private DcMotor motorLeftB;
    private DcMotor motorRightF;
    private DcMotor motorRightB;
    Servo ServoL;
    Servo ServoR;
    ColorSensor BeaconSensor;

    String blue = "blue";
    String red = "red";
    String sColor = "";

    @Override
    public void runOpMode() throws InterruptedException {

        motorLeftB = hardwareMap.dcMotor.get("motorLeftB");
        motorLeftF = hardwareMap.dcMotor.get("motorLeftF");
        motorRightB = hardwareMap.dcMotor.get("motorRightB");
        motorRightF = hardwareMap.dcMotor.get("motorRightF");
        motorLeftB.setDirection(DcMotor.Direction.REVERSE);
        motorLeftF.setDirection(DcMotor.Direction.REVERSE);
        ServoL = hardwareMap.servo.get("ServoL");
        ServoR = hardwareMap.servo.get("ServoR");
        BeaconSensor = hardwareMap.colorSensor.get("BeaconSensor");
        BeaconSensor.enableLed(false);

        waitForStart();
        while (opModeIsActive()) {
           // BeaconSensor.enableLed(false);
            RevisarColor();
            if (sColor.equals(blue)) {
                Avanzar(.1);
                sleep(1000);
                Frenar();
                ServoR.setPosition(.5);
                ServoL.setPosition(0);
                Reversa(.1);
                sleep(1200);
                Frenar();
                requestOpModeStop();
            }
            else {
                Avanzar(.1);
                sleep(1000);
                Frenar();
                ServoR.setPosition(1);
                ServoL.setPosition(.6);
                Reversa(.1);
                sleep(1200);
                Frenar();
                requestOpModeStop();
            }
        }
    }


    public void RevisarColor() {
        float hsvValues[] = {0, 0, 0};

        Color.RGBToHSV(BeaconSensor.red() * 8, BeaconSensor.green() * 8, BeaconSensor.blue() * 8, hsvValues);
        if (BeaconSensor.blue() > BeaconSensor.red() && BeaconSensor.blue() > BeaconSensor.green()) {
            telemetry.addData("Color", blue);
            telemetry.update();
            sColor = blue;
        } else if (BeaconSensor.red() > BeaconSensor.blue() && BeaconSensor.red() > BeaconSensor.green()) {
            telemetry.addData("Color", red);
            telemetry.update();
            sColor = red;
        }


    }

    public void PicarBeacon() {
        RevisarColor();
        if (sColor.equals(blue)) {
            ServoR.setPosition(1);
            ServoL.setPosition(.6);
        } else if (sColor.equals(red)) {
            ServoL.setPosition(0);
            ServoR.setPosition(.4);
            RevisarColor();
        } else {
            ServoL.setPosition(.6);
            ServoR.setPosition(.4);
        }
    }
    public void Reversa(double power) {
        motorRightB.setPower(power);
        motorRightF.setPower(power);
        motorLeftB.setPower(power);
        motorLeftF.setPower(power);
    }
    public void Frenar() {
        Avanzar(0);
        Reversa(0);
    }
    public void Avanzar(double power) {
        motorRightB.setPower(-power);
        motorRightF.setPower(-power);
        motorLeftB.setPower(-power);
        motorLeftF.setPower(-power);
    }
}