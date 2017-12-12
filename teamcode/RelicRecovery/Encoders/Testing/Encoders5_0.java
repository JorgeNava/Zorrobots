package org.firstinspires.ftc.teamcode.RelicRecovery.Encoders.Testing;

import android.text.style.ForegroundColorSpan;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

import java.sql.RowId;

/**
 * Created by Jorge on 01/12/2017.
 */
@TeleOp(name ="Encoders5_0",group = "TeleOp")
public class Encoders5_0 extends LinearOpMode {
    private DcMotor motor1 = null;

    @Override
    public void runOpMode() throws InterruptedException {
        motor1 = hardwareMap.dcMotor.get("motor1");
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //ANDYMARK TICKS PER REVOLUTIONS 1120

        waitForStart();
        telemetry.addLine("Starting encoders");
        telemetry.update();
       // ForwardToPosition(1, 2);
        ForwardDistance(1,10);

        StopDriving();
        telemetry.addLine("Encoders Finished");
        telemetry.update();
    }

    protected void ForwardDistance(double power, int rotations){
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setTargetPosition(rotations*1120);
        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Forward(power);
        while(motor1.isBusy()){
            telemetry.addLine("Encoders working");
            telemetry.addData("Rotation: ",motor1.getCurrentPosition()/1120);
            telemetry.update();
        }
        StopDriving();
        //motor1.setMode();
    }
    private void Forward(double power){
        motor1.setPower(power);
    }
    private void Reverse(double power){
        Forward(-power);
    }
    private void StopDriving(){
        Forward(0);
    }
}
