package org.firstinspires.ftc.team6818;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

/**
 * Created by wdhoward on 12/3/16.
 */

@TeleOp(name="AimBot: Teleop Record", group="Aimbot")
public class AimbotTeleopRecord extends AimbotTeleop {

    private String LOG_TAG = "AIMBOT RECORD - ";

    @Override
    public void start(){
        telemetry.addData("Status", "starting with recording enabled");
        telemetry.update();

        robot.startRobot();
        robot.startRecording();
        RobotLog.i(LOG_TAG + "op mode has been started with recoding turned on.");
    }

    @Override
    public void stop(){

        telemetry.addData("Status", "stopping");
        telemetry.update();

        robot.stopRecording();
        robot.stopRobot();
        RobotLog.i(LOG_TAG + "op mode has been stopped and recording turned off.");

    }

    @Override
    public void loop()
    {
        super.loop();
        robot.updateRecording();
    }
}
