
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

public class KAOS_AutoCore
{
    private BaseHardware m_robot;
    private CoreConfig m_coreConfig;

    private ElapsedTime runtime = new ElapsedTime();

    public KAOS_AutoCore(CoreConfig coreConfig)
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

        // initialize the BaseVuMark reader
        m_coreConfig.Telemetry.addData(">", "Initializing VuMark...");
        m_coreConfig.Telemetry.update();
        BaseVuMark vuMark = new BaseVuMark(m_coreConfig, true);
        RelicRecoveryVuMark glyphColumn = vuMark.ReadVuMark(2, true);

        // Wait for the game to start (driver presses PLAY)
        m_coreConfig.Telemetry.addData(">", "Waiting for start...");
        m_coreConfig.Telemetry.update();
        m_coreConfig.OpMode.waitForStart();

        // close gripper
        m_coreConfig.Telemetry.addData(">", "Closing lift gripper..");
        m_coreConfig.Telemetry.update();
        m_robot.CloseLiftGripper();

        // raise lift with glyph in it
        m_coreConfig.Telemetry.addData(">", "Raising glyph...");
        m_coreConfig.Telemetry.update();
        m_robot.RunLiftToTargetPosition(0.7, 1500, 4);

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
            m_coreConfig.Telemetry.addData(">", "Could not determine BaseVuMark position, defaulting to right...");
            m_coreConfig.Telemetry.update();
            glyphColumn = RelicRecoveryVuMark.CENTER;
        }

        // lower the jewel arm
        m_coreConfig.Telemetry.addData(">", "Lowering jewel arm...");
        m_coreConfig.Telemetry.update();
        m_robot.LowerJewelArm();
        m_coreConfig.OpMode.sleep(1000);

        m_coreConfig.Telemetry.addData(">", "Reading color...");
        m_coreConfig.Telemetry.update();
        BaseHardware.COLOR_RESULT colorResult = m_robot.ReadJewelColor(1, true);
        m_robot.DisableColorSensorLEDs();

        if ((m_coreConfig.StartingPosition == CoreConfig.ROBOT_STARTING_POSITION.RED_LEFT_BACK_1) ||
                (m_coreConfig.StartingPosition == CoreConfig.ROBOT_STARTING_POSITION.RED_LEFT_FRONT_2))
        {
            if (colorResult == BaseHardware.COLOR_RESULT.BLUE)
            {
                m_coreConfig.Telemetry.addData(">", "In position 1/2, pushing blue ball...");
                m_coreConfig.Telemetry.update();
                m_coreConfig.DriveTrainSet = CoreConfig.ROBOT_DRIVE_TRAIN_SET.A_LF_RR;
                m_robot.DriveByEncoder(.4, -15, 15, 5);
                m_robot.RaiseJewelArm();
            }
            else if (colorResult == BaseHardware.COLOR_RESULT.RED)
            {
                m_coreConfig.Telemetry.addData(">", "In position 1/2, pushing red ball...");
                m_coreConfig.Telemetry.update();
                m_coreConfig.DriveTrainSet = CoreConfig.ROBOT_DRIVE_TRAIN_SET.B_RF_LR;
                m_robot.DriveByEncoder(.4, 15, -15, 5);
                m_robot.RaiseJewelArm();
            }
            else
            {
                m_coreConfig.Telemetry.addData(">", "Raising arm, unknown color...");
                m_coreConfig.Telemetry.update();
                m_robot.RaiseJewelArm();
                m_coreConfig.DriveTrainSet = CoreConfig.ROBOT_DRIVE_TRAIN_SET.A_LF_RR;
                m_robot.DriveByEncoder(.4, -15, 15, 5);
            }
        }
        else
        {
            if (colorResult == BaseHardware.COLOR_RESULT.BLUE)
            {
                m_coreConfig.Telemetry.addData(">", "In position 3/4, pushing blue ball...");
                m_coreConfig.Telemetry.update();
                m_coreConfig.DriveTrainSet = CoreConfig.ROBOT_DRIVE_TRAIN_SET.B_RF_LR;
                m_robot.DriveByEncoder(.4, 15, -15, 5);
                m_robot.RaiseJewelArm();
            }
            else if (colorResult == BaseHardware.COLOR_RESULT.RED)
            {
                m_coreConfig.Telemetry.addData(">", "In position 3/4, pushing red ball...");
                m_coreConfig.Telemetry.update();
                m_coreConfig.DriveTrainSet = CoreConfig.ROBOT_DRIVE_TRAIN_SET.A_LF_RR;
                m_robot.DriveByEncoder(.4, -15, -15, 5);
                m_robot.RaiseJewelArm();
            }
            else
            {
                m_coreConfig.Telemetry.addData(">", "Raising arm, unknown color...");
                m_coreConfig.Telemetry.update();
                m_robot.RaiseJewelArm();
                m_coreConfig.DriveTrainSet = CoreConfig.ROBOT_DRIVE_TRAIN_SET.B_RF_LR;
                m_robot.DriveByEncoder(.4, 15, -15, 5);
            }
        }

        // what is our starting position, run the correct code
        m_coreConfig.Telemetry.update();
        if (m_coreConfig.StartingPosition == CoreConfig.ROBOT_STARTING_POSITION.RED_LEFT_BACK_1)
        {
            ExecuteAutonomousFrom_StartPos_1(glyphColumn, colorResult);
        }
        else if (m_coreConfig.StartingPosition == CoreConfig.ROBOT_STARTING_POSITION.RED_LEFT_FRONT_2)
        {
            ExecuteAutonomousFrom_StartPos_2(glyphColumn, colorResult);
        }
        else if (m_coreConfig.StartingPosition == CoreConfig.ROBOT_STARTING_POSITION.BLUE_RIGHT_FRONT_3)
        {
            ExecuteAutonomousFrom_StartPos_3(glyphColumn, colorResult);
        }
        else if (m_coreConfig.StartingPosition == CoreConfig.ROBOT_STARTING_POSITION.BLUE_RIGHT_BACK_4)
        {
            ExecuteAutonomousFrom_StartPos_4(glyphColumn, colorResult);
        }

        // drive forward to crypto box x inches
        m_coreConfig.Telemetry.addData(">", "Driving toward crypto box..");
        m_coreConfig.Telemetry.update();
        m_coreConfig.DriveTrainSet = CoreConfig.ROBOT_DRIVE_TRAIN_SET.C_ALL_FORWARD_BACKWARD;
        m_robot.DriveByEncoder(0.7, 12 , 12, 5);

        // open gripper & release glyph
        m_coreConfig.Telemetry.addData(">", "Opening gripper..");
        m_coreConfig.Telemetry.update();
        m_robot.OpenLiftGripper();

        // back away from block
        m_coreConfig.Telemetry.addData(">", "Backup away from block...");
        m_coreConfig.DriveTrainSet = CoreConfig.ROBOT_DRIVE_TRAIN_SET.C_ALL_FORWARD_BACKWARD;
        m_coreConfig.Telemetry.update();
        m_robot.DriveByEncoder(0.6, -3 , -3, 2);
    }

    public void ExecuteAutonomousFrom_StartPos_1(RelicRecoveryVuMark glyphColumn, BaseHardware.COLOR_RESULT colorResult)
    {
        m_robot.RaiseJewelArm();

        if (colorResult == BaseHardware.COLOR_RESULT.BLUE)
        {
            m_coreConfig.DriveTrainSet = CoreConfig.ROBOT_DRIVE_TRAIN_SET.B_RF_LR;
            if (glyphColumn == RelicRecoveryVuMark.LEFT)
            {
                m_robot.DriveByEncoder(.4, 3, -3, 5);
            }
            else if (glyphColumn == RelicRecoveryVuMark.CENTER)
            {
                m_robot.DriveByEncoder(.4, 9.5, -9.5, 5);
            }
            else
            {
                m_robot.DriveByEncoder(.4, 16, -16, 5);
            }
        }
        else  if (colorResult == BaseHardware.COLOR_RESULT.RED)
        {
            m_coreConfig.DriveTrainSet = CoreConfig.ROBOT_DRIVE_TRAIN_SET.A_LF_RR;
            if (glyphColumn == RelicRecoveryVuMark.LEFT)
            {
                m_robot.DriveByEncoder(.4, 3, -3, 5);
            }
            else if (glyphColumn == RelicRecoveryVuMark.CENTER)
            {
                m_robot.DriveByEncoder(.4, 9.5, -9.5, 5);
            }
            else
            {
                m_robot.DriveByEncoder(.4, 9.5, -9.5, 5);
            }
        }
    }

    public void ExecuteAutonomousFrom_StartPos_2(RelicRecoveryVuMark glyphColumn, BaseHardware.COLOR_RESULT colorResult)
    {

    }

    public void ExecuteAutonomousFrom_StartPos_3(RelicRecoveryVuMark glyphColumn, BaseHardware.COLOR_RESULT colorResult)
    {

    }

    public void ExecuteAutonomousFrom_StartPos_4(RelicRecoveryVuMark glyphColumn, BaseHardware.COLOR_RESULT colorResult)
    {

    }
}
