package org.firstinspires.ftc.teamcode.VelocityVortex.ColorSensor;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;

/**
 * Created by Jorge on 15/12/2016.
 */
//@TeleOp(name ="TelemetryAllColor")
public class TelemetryColor extends OpMode {

    String white = "white";
    String blue = "blue";
    String red = "red";
    String green = "green";
    String black = "black";

    ColorSensor BeaconSensor;
    ColorSensor LineSensor;

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
        telemetry.addData("Azul", BeaconSensor.blue());
        telemetry.addData("Rojo", BeaconSensor.red());
        telemetry.addData("Verde", BeaconSensor.green());
    }
}
