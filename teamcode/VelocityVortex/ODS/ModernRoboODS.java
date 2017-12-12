package org.firstinspires.ftc.teamcode.VelocityVortex.ODS;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

/**
 * Created by Jorge on 28/12/2016.
 */
    /*
Modern Robotics ODS Example with range approximation
Created 8/4/2016 by Colton Mehlhoff of Modern Robotics using FTC SDK 1.6
Reuse permitted with credit where credit is due

Configuration:
Optical Distance Sensor connected to an Analog port on a Core Device Interface and named "ods" in the configuration file
Logitech F310 Gamepad

This program can be run without a battery and Power Destitution Module

For more information, visit modernroboticsedu.com.
Support is available by emailing support@modernroboticsinc.com.
*/
//@TeleOp(name = "Telemetry ODS", group = "Sensor")
    public class ModernRoboODS extends LinearOpMode {

        //Instance of OpticalDistanceSensor
        OpticalDistanceSensor ODS;

        //rawValue reading when sensor is 2cm from object
        //this value can be set by pressing A on the Logitech controller
        static double reading2cm = 800;

        //rawValue reading when sensor is 10cm from object
        //this value can be set by pressing A on the Logitech controller
        static double reading10cm = 20;

        //Raw value is a whole number between 0 and 1023
        static double odsReadngRaw;
        // odsReadinRaw to the power of (-0.5)
        static double odsReadingLinear;
        // odsReading Linear scaled to match 2cm and 10cm
        static int odsEstimatedDistance;

        //y = mx + b; scaling the output to represent distance
        static double m = 56;
        static double b = -0.756;

        @Override
        public void runOpMode() throws InterruptedException {

            //identify the port of the ODS in the configuration file
            ODS = hardwareMap.opticalDistanceSensor.get("WallSensor");

            telemetry.addData("0 Status", "Waiting for Start");
            waitForStart();

            while (opModeIsActive()) {

                if (gamepad1.a) { //if a is pressed, update 2cm and recalculate m and b
                    telemetry.addData("0 Status", "Updating 2 cm");
                    reading2cm = ODS.getLightDetected();
                    calculateMB();
                    while (gamepad1.a) {
                    }

                }
                if (gamepad1.b) { //if b is pressed, update 10cm and recalculate m and b
                    telemetry.addData("0 Status", "Updating 10 cm");
                    reading10cm = ODS.getRawLightDetected();
                    calculateMB();
                    while (gamepad1.b) {
                    }
                }

                odsReadngRaw = ODS.getRawLightDetected();                       //update raw value
                odsReadingLinear = Math.pow(odsReadngRaw, -0.5);                //calculate linear value
                odsEstimatedDistance = (int) ((m * odsReadingLinear) + b);      //estimate distance

                telemetry.addData("0 Status", "Normal");
                telemetry.addData("2 ODS Raw", odsReadngRaw);
                telemetry.addData("3 Distance", odsEstimatedDistance);
                telemetry.addData("4 m", m);
                telemetry.addData("5 b", b);
                telemetry.update();
                waitOneFullHardwareCycle();
            }
        }

        private static void calculateMB() {
            m = 8 / (Math.pow((double)reading10cm, -0.5) - Math.pow((double)reading2cm, -0.5));
            b = 2 - Math.pow(reading2cm, -0.5) * m;
        }

    }//end class
