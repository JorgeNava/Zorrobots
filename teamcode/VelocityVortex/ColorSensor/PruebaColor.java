package org.firstinspires.ftc.teamcode.VelocityVortex.ColorSensor;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;

/**
 * Created by Jorge on 15/12/2016.
 */
//@TeleOp(name ="TelemetryColor")
public class PruebaColor extends OpMode {

    String white = "white";
    String blue = "blue";
    String red = "red";
    String green = "green";
    String black = "black";

    ColorSensor BeaconSensor;

    @Override
    public void init() {

        BeaconSensor = hardwareMap.colorSensor.get("BeaconSensor");
        BeaconSensor.setI2cAddress(I2cAddr.create7bit(0x26));//4c
        BeaconSensor.enableLed(false);

    }

    @Override
    public void loop() {

        BeaconSensor.enableLed(false);
        float hsvValues[] = {0,0,0};


            Color.RGBToHSV(BeaconSensor.red()*8, BeaconSensor.green()*8,BeaconSensor.blue()*8,hsvValues);
        if (BeaconSensor.blue()>0 && BeaconSensor.red()> 0 && BeaconSensor.green() > 0){
            telemetry.addData("Color", white);
        }
        else if (BeaconSensor.blue()>BeaconSensor.red()&& BeaconSensor.blue()>BeaconSensor.green()){
            telemetry.addData("Color", blue);
            }
        else if (BeaconSensor.red()>BeaconSensor.blue()&& BeaconSensor.red()>BeaconSensor.green()){
            telemetry.addData("Color", red);
        }
        else if (BeaconSensor.green()>BeaconSensor.blue()&& BeaconSensor.green()>BeaconSensor.red()){
            telemetry.addData("Color", green);
        }
        else {
            telemetry.addData("Color", black);
        }
    }
}
