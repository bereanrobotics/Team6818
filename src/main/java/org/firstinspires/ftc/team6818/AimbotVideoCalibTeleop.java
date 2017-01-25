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

import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import edu.berean.robotics.robots.team6818.HardwareDoppleBotAimbot;

/**
 * This OpMode illustrates the basics of using the Vuforia localizer to determine
 * positioning and orientation of robot on the FTC field.
 * The code is structured as a LinearOpMode
 *
 * Vuforia uses the phone's camera to inspect it's surroundings, and attempt to locate target images.
 *
 * When images are located, Vuforia is able to determine the position and orientation of the
 * image relative to the camera.  This sample code than combines that information with a
 * knowledge of where the target images are on the field, to determine the location of the camera.
 *
 * This example assumes a "diamond" field configuration where the red and blue alliance stations
 * are adjacent on the corner of the field furthest from the audience.
 * From the Audience perspective, the Red driver station is on the right.
 * The two vision target are located on the two walls closest to the audience, facing in.
 * The Stones are on the RED side of the field, and the Chips are on the Blue side.
 *
 * A final calculation then uses the location of the camera on the robot to determine the
 * robot's location and orientation on the field.
 *
 * @see VuforiaLocalizer
 * @see VuforiaTrackableDefaultListener
 * see  ftc_app/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */

@TeleOp(name="Aimbot Video Teleop", group ="Demo")
//@Disabled
public class AimbotVideoCalibTeleop extends OpMode {

    public static final String TAG = "Aimbot Video Teleop";

    //INITIALIZE TELEOP VARIABLES

    double left;
    double right;
    double launcherpower;
    double spinnerpower;
    double RightButtonPosition;
    double LeftButtonPosition;

    //INITIALIZE VIDEO NAVIGATION VARIABLES
    OpenGLMatrix lastLocation = null;
    List<VuforiaTrackable> allTrackables = null;
    VuforiaTrackables targets = null;

    protected HardwareDoppleBotAimbot robot = new HardwareDoppleBotAimbot(); // use the class created to define a Aimbot's hardware
    protected boolean sniperModeOn = true;

    private String LOG_TAG = "AIMBOT VIDEO TELEOP - ";

    @Override public void init() {

        //INITIALIZE HARDWARE

        robot.initializeRobot(hardwareMap);

        telemetry.addData("Say", "Hello Driver");    //
        updateTelemetry(telemetry);
        RobotLog.i(LOG_TAG + "robot initialization completed.");

        //INITIALIZE VUFORIA SOFTWARE

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "Ac80Bxv/////AAAAGfuMJvbL4EHBo0m/sdW89cEZQtlemG/Mzd5+DnP8zVE+fzNuHuRvUQTpoNwJ6/rkPwmuMiCAzOmspCyheyCAl5OLh7Xbp95m1wWcdjb/kUAdtPsfNel0eNn2ji0iwoWuPtQJ+b8YLFSyzBCkHe9dB05c86KmdvJHAH7+y//rLsAEy5/josOhopghBAEJGUjjwWe5n/I0Fe2Hmpv1Kw8SuhCFp3ibSL/YoCRYrRx0lKQSQ7ZdlcEVO2uVAMpStijUn1rWNlp6WaaZ5+Z1Su8IVWRll/5xFZGs/vdcxpqbJbGXKfZsQE1vYSH6rPc/w0cvBJW09GHxnburccDC5ZBTva63VGbAhBWlkp/MXUvlN9Q5";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        targets = this.vuforia.loadTrackablesFromAsset("FTC_2016-17");
        VuforiaTrackable redTarget1 = targets.get(3);
        redTarget1.setName("RedTarget1");  // Gears
        VuforiaTrackable redTarget2 = targets.get(1);
        redTarget2.setName("RedTarget2");  // Tools


        VuforiaTrackable blueTarget1  = targets.get(0);
        blueTarget1.setName("BlueTarget1");  // Legos
        VuforiaTrackable blueTarget2  = targets.get(2);
        blueTarget2.setName("BlueTarget2");  // Wheels

        allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targets);

        float mmPerInch        = 25.4f;
        float mmBotWidth       = 18 * mmPerInch;            // ... or whatever is right for your robot
        float mmFTCFieldWidth  = (12*12 - 2) * mmPerInch;   // the FTC field is ~11'10" center-to-center of the glass panels

        //Set Red Target 1 Location: Gears
        OpenGLMatrix redTargetLocationOnField1 = OpenGLMatrix
                /* Then we translate the target off to the RED WALL. Our translation here
                is a negative translation in X.*/
                .translation(-mmFTCFieldWidth / 2, -mmFTCFieldWidth / 12, 0)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X, then 90 in Z */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0));
        redTarget1.setLocation(redTargetLocationOnField1);
        RobotLog.ii(TAG, "Red Target 1=%s", redTargetLocationOnField1.formatAsTransform());

        //Set Red Target 2 Location: Tools
        OpenGLMatrix redTargetLocationOnField2 = OpenGLMatrix
                .translation(-mmFTCFieldWidth/2, mmFTCFieldWidth/4, 0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0));
        redTarget2.setLocation(redTargetLocationOnField2);
        RobotLog.ii(TAG, "Red Target 2=%s", redTargetLocationOnField2.formatAsTransform());

        //Set Blue Target 1 Location: Wheels
        OpenGLMatrix blueTargetLocationOnField1 = OpenGLMatrix
                /* Then we translate the target off to the Blue Audience wall.
                Our translation here is a positive translation in Y.*/
                .translation(mmFTCFieldWidth/12, mmFTCFieldWidth/2, 0)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));
        blueTarget1.setLocation(blueTargetLocationOnField1);
        RobotLog.ii(TAG, "Blue Target 1=%s", blueTargetLocationOnField1.formatAsTransform());

        //Set Blue Target 2 Location: Legos
        OpenGLMatrix blueTargetLocationOnField2 = OpenGLMatrix
                .translation(-mmFTCFieldWidth/4, mmFTCFieldWidth/2, 0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));
        blueTarget2.setLocation(blueTargetLocationOnField2);
        RobotLog.ii(TAG, "Blue Target 2=%s", blueTargetLocationOnField2.formatAsTransform());

        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(0,mmBotWidth/4,0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.YZY,
                        AngleUnit.DEGREES, 0, 180, 0));
        RobotLog.ii(TAG, "phone=%s", phoneLocationOnRobot.formatAsTransform());

        ((VuforiaTrackableDefaultListener)redTarget1.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)redTarget2.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)blueTarget1.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)blueTarget2.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
    }

    @Override public void init_loop() {

    }

    @Override public void start() {
        RobotLog.i(LOG_TAG + "op mode has been started.");
    }

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;

    @Override public void loop() {

        //OPERATE VIDEO TRACKING
        targets.activate();


            for (VuforiaTrackable trackable : allTrackables) {
                /**
                 * getUpdatedRobotLocation() will return null if no new information is available since
                 * the last time that call was made, or if the trackable is not currently visible.
                 * getRobotLocation() will return null if the trackable is not currently visible.
                 */
                telemetry.addData(trackable.getName(), ((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible() ? "Visible" : "Not Visible");    //

                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
            }
            /**
             * Provide feedback as to where the robot was last located (if we know).
             */
            if (lastLocation != null) {
                //  RobotLog.vv(TAG, "robot=%s", format(lastLocation));
                /*
                telemetry.addData("Pitch: ", lastLocation.get(0,1)); //Rotation about X axis
                telemetry.addData("Roll:  ", lastLocation.get(0,0)); //Rotation about Y axis
                telemetry.addData("Yaw:   ", lastLocation.get(0,2)); //Rotation about Z axis
                telemetry.addData("X: ", lastLocation.get(1,0));
                telemetry.addData("Y: ", lastLocation.get(1,1));
                telemetry.addData("Z: ", lastLocation.get(1,2));
                */
                telemetry.addData("Pos: ", lastLocation.formatAsTransform());
            } else {
                telemetry.addLine("Position Unknown");
        }
        telemetry.update();

        //OPERATE HARDWARE

        left = gamepad1.left_stick_y;
        right = gamepad1.right_stick_y;
        spinnerpower = gamepad2.right_stick_y;
        launcherpower = gamepad2.left_stick_y;
        RightButtonPosition = gamepad2.right_stick_x;
        LeftButtonPosition = gamepad2.left_stick_x;

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
        robot.spinner.setPower(spinnerpower);
        robot.launcher.setPower(launcherpower);
        robot.rightButtonPusher.setPosition(RightButtonPosition);
        robot.leftButtonPusher.setPosition(LeftButtonPosition);
    }
}
