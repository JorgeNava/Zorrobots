package org.firstinspires.ftc.teamcode.RelicRecovery.Encoders.Telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Jorge on 01/12/2017.
 */
@TeleOp(name ="One_Encoder_Telemetry",group = "TeleOp")
public class One_Encoder_Telemetry extends LinearOpMode {
    private DcMotor motor1 = null;

    @Override
    public void runOpMode() throws InterruptedException {
        motor1 = hardwareMap.dcMotor.get("motor1");
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //ANDYMARK TICKS PER REVOLUTIONS 1120

        waitForStart();
        while (opModeIsActive()){telemetry.addData("Tics: ",motor1.getCurrentPosition());
            telemetry.update();}

    }
    private void ForwardToPosition(double power, int rotations){
        /*motor1.setMode(DcMotor.RunMode.RESET_ENCODERS);
        Left_motors.setMode(DcMotor.RunMode.RESET_ENCODERS);*/
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor1.setTargetPosition(rotations*1120);

        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Forward(power);

        while(motor1.isBusy()){
            telemetry.addLine("Encoders working");
            telemetry.update();
        }

        StopDriving();
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
