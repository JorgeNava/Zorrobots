/* Copyright (c) 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.teamcode.VelocityVortex.ODS;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

/*
 * This is an example LinearOpMode that shows how to use
 * a Modern Robotics Optical Distance Sensor
 * It assumes that the ODS sensor is configured with a name of "sensor_ods".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
//@TeleOp(name = "PruebaODS", group = "Sensor")
//@Disabled
public class SensorMROpticalDistance extends LinearOpMode {

  OpticalDistanceSensor odsSensor;  // Hardware Device Object
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
  static double m = 56;
  static double b = -0.356;



  @Override
  public void runOpMode() {

    // get a reference to our Light Sensor object.
    odsSensor = hardwareMap.opticalDistanceSensor.get("WallSensor");

    // wait for the start button to be pressed.
    waitForStart();

    // while the op mode is active, loop and read the light levels.
    // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
    while (opModeIsActive()) {

      odsReadngRaw = odsSensor.getRawLightDetected();                       //update raw value
      odsReadingLinear = Math.pow(odsReadngRaw, -0.5);                //calculate linear value
      odsEstimatedDistance = (int) ((m * odsReadingLinear) + b);      //estimate distance

      // send the info back to driver station using telemetry function.
      telemetry.addData("Raw",    odsSensor.getRawLightDetected());
      telemetry.addData("Normal", odsSensor.getLightDetected());
      telemetry.addData("Distance", odsEstimatedDistance);

      telemetry.update();
    }
  }
  private static void calculateMB() {
    m = 8 / (Math.pow((double)reading10cm, -0.5) - Math.pow((double)reading2cm, -0.5));
    b = 2 - Math.pow(reading2cm, -0.5) * m;
  }
}
