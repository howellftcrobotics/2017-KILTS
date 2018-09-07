
package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

public class KAOS_AutoCoreDirect
{
    private BaseHardware m_robot;
    private CoreConfig m_coreConfig;

    public KAOS_AutoCoreDirect(CoreConfig coreConfig)
    {
        m_coreConfig = coreConfig;
        m_robot = new BaseHardware(m_coreConfig);
    }

    public void runOpMode()
    {
        if (!m_robot.init())
        {
            m_coreConfig.Telemetry.addData("Error", "Hardware Init Failed");
            m_coreConfig.Telemetry.update();
            m_coreConfig.OpMode.sleep(8000);  // sleep for 8 seconds so the error can be read
            return;
        }

        // initialize the devices
        m_robot.InitLiftGripper();
        m_robot.InitRelicGripper();
        m_robot.InitRelicArmKnuckle();
        m_robot.InitializeGryo();
        m_robot.RaiseLeftJewelArm();
        m_robot.RaiseRightJewelArm();
        m_robot.CenterRightJewelArmFlicker();
        m_robot.CenterLeftJewelArmFlicker();

        // initialize the BaseVuMark reader
        m_coreConfig.Telemetry.addData(">", "Initializing VuMark...");
        m_coreConfig.Telemetry.update();
        BaseVuMark vuMark = new BaseVuMark(m_coreConfig, true);
        RelicRecoveryVuMark glyphColumn = vuMark.ReadVuMark(1, true);

        // Wait for the game to start (driver presses PLAY)
        m_coreConfig.Telemetry.addData(">", "Waiting for start..." + glyphColumn.toString());
        m_coreConfig.Telemetry.update();
        m_coreConfig.OpMode.waitForStart();

        // start vacuum and close gripper
        m_coreConfig.Telemetry.addData(">", "Starting vacuum and closing lift gripper..");
        m_coreConfig.Telemetry.update();
        m_robot.RunVacuumMotor(1);
        m_coreConfig.OpMode.sleep(1000);
        m_robot.CloseLiftGripper();
        m_coreConfig.OpMode.sleep(500);

        // raise lift with glyph in it
        m_coreConfig.Telemetry.addData(">", "Raising glyph...");
        m_coreConfig.Telemetry.update();
        m_robot.RunLiftToTargetPosition(0.8, -700, 4);
        //m_robot.RunLiftByTime(-.7, 1.2);

        // read the BaseVuMark for a few seconds
        if (glyphColumn == RelicRecoveryVuMark.UNKNOWN)
        {
            m_coreConfig.Telemetry.addData(">", "VuMark unknown, reading...");
            m_coreConfig.Telemetry.update();
            glyphColumn = vuMark.ReadVuMark(2, true);
            vuMark.Deactivate();
        }

        // if we couldn't read the vumark, default to right
        if (glyphColumn == RelicRecoveryVuMark.UNKNOWN)
        {
            m_coreConfig.Telemetry.addData(">", "Could not determine BaseVuMark position, defaulting to center...");
            m_coreConfig.Telemetry.update();
            glyphColumn = RelicRecoveryVuMark.CENTER;
            //glyphColumn = RelicRecoveryVuMark.RIGHT;
            //glyphColumn = RelicRecoveryVuMark.LEFT;
        }

        // lower the jewel arm
        m_coreConfig.Telemetry.addData(">", "Lowering jewel arm...");
        m_coreConfig.Telemetry.update();
        m_robot.LowerJewelArm();
        m_coreConfig.OpMode.sleep(1500);

        // read the color
        m_coreConfig.Telemetry.addData(">", "Reading color...");
        m_coreConfig.Telemetry.update();
        BaseHardware.COLOR_RESULT colorResult = m_robot.ReadJewelColor(2, true);
        m_robot.DisableColorSensorLEDs();
        m_coreConfig.Telemetry.addData(">", "Read color: " + colorResult.toString());
        m_coreConfig.Telemetry.update();
        m_coreConfig.OpMode.sleep(500);

        // knock off the jewel
        if ((m_coreConfig.StartingPosition == CoreConfig.ROBOT_STARTING_POSITION.RED_LEFT_BACK_1) ||
            (m_coreConfig.StartingPosition == CoreConfig.ROBOT_STARTING_POSITION.RED_LEFT_FRONT_2))
        {
            if (colorResult == BaseHardware.COLOR_RESULT.RED)
            {
                m_coreConfig.Telemetry.addData(">", "Pos 1/2, saw blue, pull arm");
                m_coreConfig.Telemetry.update();
                m_coreConfig.OpMode.sleep(500);
                m_robot.PullJewelArmFlicker();
            }
            else if (colorResult == BaseHardware.COLOR_RESULT.BLUE)
            {
                m_coreConfig.Telemetry.addData(">", "Pos 1/2, saw red, push arm");
                m_coreConfig.Telemetry.update();
                m_coreConfig.OpMode.sleep(500);
                m_robot.PushJewelArmFlicker();
            }
            else
            {
                m_coreConfig.Telemetry.addData(">", "Pos 1/2, saw nothing, do nothing");
                m_coreConfig.Telemetry.update();
                m_coreConfig.OpMode.sleep(500);
                m_robot.CenterJewelArmFlicker();
            }
        }
        else
        {
            if (colorResult == BaseHardware.COLOR_RESULT.BLUE)
            {
                m_coreConfig.Telemetry.addData(">", "Pos 3/4, saw blue, pull arm");
                m_coreConfig.Telemetry.update();
                m_coreConfig.OpMode.sleep(500);
                m_robot.PullJewelArmFlicker();
            }
            else if (colorResult == BaseHardware.COLOR_RESULT.RED)
            {
                m_coreConfig.Telemetry.addData(">", "Pos 3/4, saw red, push arm");
                m_coreConfig.Telemetry.update();
                m_coreConfig.OpMode.sleep(500);
                m_robot.PushJewelArmFlicker();
            }
            else
            {
                m_coreConfig.Telemetry.addData(">", "Pos 3/4, saw nothing, did nothing");
                m_coreConfig.Telemetry.update();
                m_coreConfig.OpMode.sleep(500);
                m_robot.CenterJewelArmFlicker();
            }
        }

        // after we are done, center and raise the jewel arm before moving
        m_coreConfig.Telemetry.addData(">", "Centering flicker & raising arm...");
        m_coreConfig.Telemetry.update();
        m_robot.RaiseJewelArm();
        m_coreConfig.OpMode.sleep(1000);
        m_robot.CenterJewelArmFlicker();
        m_coreConfig.OpMode.sleep(1000);

        // what is our starting position, run the correct code
        m_coreConfig.Telemetry.update();
        if (m_coreConfig.StartingPosition == CoreConfig.ROBOT_STARTING_POSITION.RED_LEFT_BACK_1)
        {
            // drive straight off
            m_coreConfig.DriveTrainSet = CoreConfig.ROBOT_DRIVE_TRAIN_SET.C_ALL_FORWARD_BACKWARD;
            m_robot.DriveByEncoder(0.9, 50 , 50, 7);
            m_robot.gyroTurn(.3, 1, 2);
            m_robot.gyroHold(.5, 1, 1);

            // drive straight (distance depends on glyph column that was read)
            m_coreConfig.DriveTrainSet = CoreConfig.ROBOT_DRIVE_TRAIN_SET.D_ALL_SIDE_2_SIDE;
            if (glyphColumn == RelicRecoveryVuMark.LEFT)
            {
                m_robot.DriveByEncoder(0.9, -37, -37, 5);
            }
            else if ((glyphColumn == RelicRecoveryVuMark.CENTER) || (glyphColumn == RelicRecoveryVuMark.UNKNOWN))
            {
                m_robot.DriveByEncoder(0.9, -23, -23, 5);
            }
            else if (glyphColumn == RelicRecoveryVuMark.RIGHT)
            {
                m_robot.DriveByEncoder(0.9, -12 , -12, 5);
            }
            m_coreConfig.OpMode.sleep(500);

            // straighten up and drive forward to crypto box
            m_coreConfig.DriveTrainSet = CoreConfig.ROBOT_DRIVE_TRAIN_SET.C_ALL_FORWARD_BACKWARD;
            m_robot.gyroTurn(.3, 1, 2);
            m_robot.gyroHold(.5, 1, 1);
            m_coreConfig.OpMode.sleep(500);
            m_robot.DriveByEncoder(0.7, 20 , 20, 5);
        }
        else if (m_coreConfig.StartingPosition == CoreConfig.ROBOT_STARTING_POSITION.RED_LEFT_FRONT_2)
        {
            // drive straight (distance depends on glyph column that was read)
            m_coreConfig.DriveTrainSet = CoreConfig.ROBOT_DRIVE_TRAIN_SET.C_ALL_FORWARD_BACKWARD;
            if (glyphColumn == RelicRecoveryVuMark.RIGHT)
            {
                m_robot.DriveByEncoder(0.9, 48, 48, 5);
            }
            else if ((glyphColumn == RelicRecoveryVuMark.CENTER) || (glyphColumn == RelicRecoveryVuMark.UNKNOWN))
            {
                m_robot.DriveByEncoder(0.9, 67, 67, 5);
            }
            else if (glyphColumn == RelicRecoveryVuMark.LEFT)
            {
                m_robot.DriveByEncoder(0.9, 86 , 86, 5);
            }
            m_coreConfig.OpMode.sleep(500);

            // straighten up and drive forward to crypto box
            m_coreConfig.DriveTrainSet = CoreConfig.ROBOT_DRIVE_TRAIN_SET.C_ALL_FORWARD_BACKWARD;
            m_robot.gyroTurn(.3, -90, 2);
            m_robot.gyroHold(.5, -90, 1);
            m_coreConfig.OpMode.sleep(500);
            m_robot.DriveByEncoder(0.7, 20 , 20, 5);
        }
        else if (m_coreConfig.StartingPosition == CoreConfig.ROBOT_STARTING_POSITION.BLUE_RIGHT_FRONT_3)
        {
            // drive straight (distance depends on glyph column that was read)
            m_coreConfig.DriveTrainSet = CoreConfig.ROBOT_DRIVE_TRAIN_SET.C_ALL_FORWARD_BACKWARD;
            if (glyphColumn == RelicRecoveryVuMark.LEFT)
            {
                m_robot.DriveByEncoder(0.9, 53, 53, 5);
            }
            else if ((glyphColumn == RelicRecoveryVuMark.CENTER) || (glyphColumn == RelicRecoveryVuMark.UNKNOWN))
            {
                m_robot.DriveByEncoder(0.9, 69, 69, 5);
            }
            else if (glyphColumn == RelicRecoveryVuMark.RIGHT)
            {
                m_robot.DriveByEncoder(0.9, 84, 84, 6);
            }
            m_coreConfig.OpMode.sleep(500);

            // straighten up and drive forward to crypto box
            m_coreConfig.DriveTrainSet = CoreConfig.ROBOT_DRIVE_TRAIN_SET.C_ALL_FORWARD_BACKWARD;
            m_robot.gyroTurn(.3, 91, 2);
            m_robot.gyroHold(.5, 91, 1);
            m_coreConfig.OpMode.sleep(500);
            m_robot.DriveByEncoder(0.7, 20 , 20, 5);
        }
        else if (m_coreConfig.StartingPosition == CoreConfig.ROBOT_STARTING_POSITION.BLUE_RIGHT_BACK_4)
        {
            // drive straight off
            m_coreConfig.DriveTrainSet = CoreConfig.ROBOT_DRIVE_TRAIN_SET.C_ALL_FORWARD_BACKWARD;
            m_robot.DriveByEncoder(0.8, 50 , 50, 7);
            m_robot.gyroTurn(.3, 0, 2);
            m_robot.gyroHold(.5, 0, 1);
            m_coreConfig.OpMode.sleep(500);

            // drive straight (distance depends on glyph column that was read)
            m_coreConfig.DriveTrainSet = CoreConfig.ROBOT_DRIVE_TRAIN_SET.D_ALL_SIDE_2_SIDE;
            if (glyphColumn == RelicRecoveryVuMark.LEFT)
            {
                m_robot.DriveByEncoder(0.9, 9.5, 9.5, 5);
            }
            else if ((glyphColumn == RelicRecoveryVuMark.CENTER) || (glyphColumn == RelicRecoveryVuMark.UNKNOWN))
            {
                m_robot.DriveByEncoder(0.9, 21, 21, 5);
            }
            else if (glyphColumn == RelicRecoveryVuMark.RIGHT)
            {
                m_robot.DriveByEncoder(0.9, 33.5 , 33.5, 5);
            }
            m_coreConfig.OpMode.sleep(500);

            // straighten up and drive forward to crypto box
            m_coreConfig.DriveTrainSet = CoreConfig.ROBOT_DRIVE_TRAIN_SET.C_ALL_FORWARD_BACKWARD;
            m_robot.gyroTurn(.3, 1, 2);
            m_robot.gyroHold(.5, 1, 1);
            m_coreConfig.OpMode.sleep(500);
            m_robot.DriveByEncoder(0.7, 20 , 20, 5);
        }

        // open gripper & release glyph
        m_coreConfig.Telemetry.addData(">", "Stop vacuum and opening gripper..");
        m_coreConfig.Telemetry.update();
        m_robot.StopVacuumMotor();
        m_robot.OpenLiftGripper();
        m_coreConfig.OpMode.sleep(1500);

        // back away from block
        m_coreConfig.Telemetry.addData(">", "Backup away from block...");
        m_coreConfig.DriveTrainSet = CoreConfig.ROBOT_DRIVE_TRAIN_SET.C_ALL_FORWARD_BACKWARD;
        m_coreConfig.Telemetry.update();
        m_robot.DriveByEncoder(0.8, -12 , -12, 2);

        m_robot.DriveByEncoder(0.9, 18 , 18, 2);
        m_robot.DriveByEncoder(0.8, -8 , -8, 2);
    }
}
