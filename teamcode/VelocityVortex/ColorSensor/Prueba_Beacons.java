package org.firstinspires.ftc.teamcode.VelocityVortex.ColorSensor;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Jorge on 15/12/2016.
 */
//@Autonomous(name ="Prueba_Beacons")
public class Prueba_Beacons extends LinearOpMode {

    Servo ServoL;
    Servo ServoR;
    ColorSensor BeaconSensor;
    ColorSensor LineSensor;

    String blue = "blue";
    String red = "red";
    String sColor="";

    @Override
    public void runOpMode() throws InterruptedException {

        ServoL = hardwareMap.servo.get("ServoL");
        ServoR = hardwareMap.servo.get("ServoR");
        BeaconSensor = hardwareMap.colorSensor.get("BeaconSensor");
        BeaconSensor.setI2cAddress(I2cAddr.create7bit(0x26));//4c

        BeaconSensor.enableLed(false);

        waitForStart();
        BeaconSensor.enableLed(false);
        while (opModeIsActive()) {
            //while (true) {
                BeaconSensor.enableLed(false);
                RevisarColor();
                if (sColor.equals(blue)) {
                    ServoR.setPosition(.5);
                    ServoL.setPosition(0);
                    RevisarColor();
                }
            else {
                    ServoR.setPosition(1);
                    ServoL.setPosition(.6);

                }
            }
        }
    public void RevisarColor(){
        float hsvValues[] = {0,0,0};

        BeaconSensor.enableLed(true);

        Color.RGBToHSV(BeaconSensor.red()*8, BeaconSensor.green()*8,BeaconSensor.blue()*8,hsvValues);
       if (BeaconSensor.blue()>BeaconSensor.red()&& BeaconSensor.blue()>BeaconSensor.green()){
            telemetry.addData("Color", blue);
            telemetry.update();
            sColor=blue;
        }
        else if (BeaconSensor.red()>BeaconSensor.blue()&& BeaconSensor.red()>BeaconSensor.green()){
            telemetry.addData("Color", red);
            telemetry.update();
            sColor=red;
        }

    }
}