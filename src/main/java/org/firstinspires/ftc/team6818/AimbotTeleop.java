/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.team6818;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import edu.berean.robotics.robots.team6818.HardwareDoppleBotAimbot;



/**
 * This file provides  Telop driving for Aimbot.
 */

@TeleOp(name="AimBot: Teleop", group="Aimbot")
// @Disabled
public class AimbotTeleop extends OpMode{

    /* Declare OpMode members. */

    protected HardwareDoppleBotAimbot robot = new HardwareDoppleBotAimbot(); // use the class created to define a Aimbot's hardware
    protected boolean sniperModeOn = true;

    private String LOG_TAG = "AIMBOT TELEOP - ";
    ElapsedTime runtime = new ElapsedTime();

    double qermyStartPos = 0.4;
    double qermyEndPos = 1;
    double qermyOffset = 0.4;
    double qermySpeed = 0.01;
    boolean delayOn = false;
    boolean readyForTimerReset = true;
    boolean needsTimerReset = true;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.initializeRobot(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        updateTelemetry(telemetry);
        RobotLog.i(LOG_TAG + "robot initialization completed.");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }



    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        RobotLog.i(LOG_TAG + "op mode has been started.");
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double left;
        double right;
        double launcherpower;
        double spinnerpower;
        double qermyOffset;
        //double pusherUpLeftPos = 1;
        //double pusherDownPos = 0.4;
        //double RightButtonPosition;
        //double LeftButtonPosition;


        left = gamepad1.left_stick_y;
        right = gamepad1.right_stick_y;
        spinnerpower = gamepad2.right_stick_x;
        launcherpower = gamepad2.left_stick_y;
        qermyOffset = gamepad2.right_trigger;
        //RightButtonPosition = gamepad2.right_stick_x;
        //LeftButtonPosition = gamepad2.left_stick_x;

        if (!sniperModeOn) {
            robot.frontLeftMotor.setPower(left);
            robot.backLeftMotor.setPower(left);
            robot.frontRightMotor.setPower(right);
            robot.backRightMotor.setPower(right);
        }
        else
        {
            robot.frontLeftMotor.setPower(left/3);
            robot.backLeftMotor.setPower(left/3);
            robot.frontRightMotor.setPower(right/3);
            robot.backRightMotor.setPower(right/3);
        }
        if (gamepad1.right_bumper)
        {
            sniperModeOn = true;
        }
        if (gamepad1.left_bumper)
        {
            sniperModeOn = false;
        }

        if (gamepad2.x && !delayOn)
        {
            delayOn = true;
        }
        if (delayOn && qermyOffset > qermyEndPos)
        {
            qermyOffset -= qermySpeed;
        }
        if (delayOn && qermyOffset <= qermyEndPos)
        {
            if (readyForTimerReset)
            {
                runtime.reset();
                readyForTimerReset = false;
            }
            if (runtime.seconds() > 1)
            {
                delayOn = false;
            }

        }
        if (qermyOffset < qermyStartPos && !delayOn)
        {
            readyForTimerReset = true;
            qermyOffset += qermySpeed;
        }

        robot.reuptake.setPosition(qermyOffset);

        robot.spinner.setPower(spinnerpower);
        robot.launcher.setPower(launcherpower);

        //robot.rightButtonPusher.setPosition(RightButtonPosition);
        //robot.leftButtonPusher.setPosition(LeftButtonPosition);


        /*
        }
        // Use gamepad left & right Bumpers to open and close the claw
        if (gamepad1.right_bumper)
            clawOffset += CLAW_SPEED;
        else if (gamepad1.left_bumper)
            clawOffset -= CLAW_SPEED;

        // Move both servos to new position.  Assume servos are mirror image of each other.
        clawOffset = Range.clip(clawOffset, -0.5, 0.5);
        robot.leftButtonPusher.setPosition(robot.MID_SERVO + clawOffset);
        robot.rightButtonPusher.setPosition(robot.MID_SERVO - clawOffset);

        // Use gamepad buttons to move the arm up (Y) and down (A)
        if (gamepad1.y)
            robot.armMotor.setPower(robot.ARM_UP_POWER);
        else if (gamepad1.a)
            robot.armMotor.setPower(robot.ARM_DOWN_POWER);
        else
            robot.armMotor.setPower(0.0);
        */

        // Send telemetry message to signify robot running;
        //telemetry.addData("claw",  "Offset = %.2f", clawOffset);
        telemetry.addData("left",  "%.2f", left);
        telemetry.addData("right", "%.2f", right);
        telemetry.addData("qermy", "%.2f", qermyOffset);
        updateTelemetry(telemetry);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        RobotLog.i(LOG_TAG + "op mode has been stopped.");
    }

}
