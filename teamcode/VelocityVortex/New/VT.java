package org.firstinspires.ftc.teamcode.VelocityVortex.New;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

/**
 * Created by Jorge on 01/02/2017.
 */

//@Autonomous(name = "VT")
public class VT extends LinearOpMode {

    public VoltageSensor vs;
    private DcMotor motorLeftF;
    private DcMotor motorLeftB;
    private DcMotor motorRightF;
    private DcMotor motorRightB;

    @Override
    public void runOpMode() throws InterruptedException {
        vs = hardwareMap.voltageSensor.get("MCwheelsR");
        motorLeftB = hardwareMap.dcMotor.get("motorLeftB");
        motorLeftF = hardwareMap.dcMotor.get("motorLeftF");
        motorRightB = hardwareMap.dcMotor.get("motorRightB");
        motorRightF = hardwareMap.dcMotor.get("motorRightF");

        waitForStart();
        double a = vs.getVoltage();
        if (a > 13){
            motorRightB.setPower(-1);
            motorRightF.setPower(-1);
            motorLeftB.setPower(1);
            motorLeftF.setPower(1);
            telemetry.addData("VOLTAGE", a);
            telemetry.update();
            sleep(2000);
        }
        else {
            motorRightB.setPower(0);
            motorRightF.setPower(0);
            motorLeftB.setPower(0);
            motorLeftF.setPower(0);
            telemetry.addData("VOLTAGE", a);
            telemetry.update();
            sleep(2000);
        }
        telemetry.addData("VOLTAGE", a);
        telemetry.update();
        sleep(100000);
    }
}
