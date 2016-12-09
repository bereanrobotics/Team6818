package org.firstinspires.ftc.team6818;

/* Copyright (c) 2014 Qualcomm Technologies Inc

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

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import java.io.File;

import edu.berean.robotics.dopple.core.DoppleBotHistoryRecord;
import edu.berean.robotics.dopple.util.DoppleBotHistoryHelper;
import edu.berean.robotics.robots.team6818.HardwareDoppleBotAimbot;

@Autonomous(name = "Aimbot: BLUE 1", group = "Aimbot")
//@Disabled

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
public class AimbotAutoBlue1 extends LinearOpMode {

    private static String ROBOT_HISTORY_DIRECTORY = "/ROBO_DATA/";
    private static String PLAY_FILENAME = "BLUE_1.txt";
    private String LOG_TAG = "AIMBOT BLUE 1 - ";

    protected HardwareDoppleBotAimbot robot = new HardwareDoppleBotAimbot();


    /**
     * Constructor
     */
    public AimbotAutoBlue1() {

    }

    /*
     * Code to run when the op mode is initialized goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#init()
     */
    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("Status", "Initializing.");
        telemetry.update();

        RobotLog.i(LOG_TAG + "initializing robot");
        robot.initializeRobot(hardwareMap);
        File historyFile = new File (Environment.getExternalStorageDirectory().getAbsolutePath() + ROBOT_HISTORY_DIRECTORY + PLAY_FILENAME);

        RobotLog.d(LOG_TAG + "file obtained for playback: %s", historyFile);

        waitForStart();

        if (historyFile.exists()){
            DoppleBotHistoryRecord historyToPlay = DoppleBotHistoryHelper.getHistoryFromFile(historyFile);
            robot.startPlayback(historyToPlay, this);
        } else RobotLog.w(LOG_TAG + String.format("Couldn't load most recent file from %s", historyFile.toString()));

        robot.stopRobot();


    }


}
