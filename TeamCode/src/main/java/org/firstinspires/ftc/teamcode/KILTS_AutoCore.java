package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

public class KILTS_AutoCore
{
    private BaseHardware m_robot;
    private CoreConfig m_coreConfig;

    static final double GYRO_TURN_SPEED = .4 ;

    public KILTS_AutoCore(CoreConfig coreConfig)
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
        m_robot.CloseVacuumValve();

        // initialize the BaseVuMark reader
        BaseVuMark vuMark = new BaseVuMark(m_coreConfig, true);
        RelicRecoveryVuMark glyphColumn = vuMark.ReadVuMark(1, true);

        // Wait for the game to start (driver presses PLAY)
        m_coreConfig.Telemetry.addData(">", "Waiting for start...");
        m_coreConfig.Telemetry.update();
        m_coreConfig.OpMode.waitForStart();

        // close gripper
        m_coreConfig.Telemetry.addData(">", "Turn on vacuum and close lift gripper..");
        m_coreConfig.Telemetry.update();
        m_robot.CloseLiftGripper();
        m_robot.CloseVacuumValve();
        m_robot.liftVacuumMotor.setPower(.95);

        // raise lift with glyph in it
        m_coreConfig.Telemetry.addData(">", "Raising glyph...");
        m_coreConfig.Telemetry.update();
        m_robot.RunLiftToTargetPosition(0.9, 700, 4);

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
        m_coreConfig.OpMode.sleep(500);

        m_coreConfig.Telemetry.addData(">", "Reading color...");
        m_coreConfig.Telemetry.update();
        BaseHardware.COLOR_RESULT colorResult = m_robot.ReadJewelColor(2, true);
        m_robot.DisableColorSensorLEDs();

        if ((m_coreConfig.StartingPosition == CoreConfig.ROBOT_STARTING_POSITION.RED_LEFT_BACK_1) ||
            (m_coreConfig.StartingPosition == CoreConfig.ROBOT_STARTING_POSITION.RED_LEFT_FRONT_2))
        {
            if (colorResult == BaseHardware.COLOR_RESULT.BLUE)
            {
                m_coreConfig.Telemetry.addData(">", "In position 1/2, pushing blue ball...");
                m_coreConfig.Telemetry.update();
                m_robot.DriveByEncoder(.7, 2, 2, 5);
                m_robot.RaiseJewelArm();
            }
            else if (colorResult == BaseHardware.COLOR_RESULT.RED)
            {
                m_coreConfig.Telemetry.addData(">", "In position 1/2, pushing red ball...");
                m_coreConfig.Telemetry.update();
                m_robot.DriveByEncoder(.7, -2.5, -2.5, 5);
                m_robot.RaiseJewelArm();
                m_robot.DriveByEncoder(.7, 5, 5, 5);
            }
            else
            {
                m_coreConfig.Telemetry.addData(">", "Raising arm, unknown color...");
                m_coreConfig.Telemetry.update();
                m_robot.RaiseJewelArm();
                m_robot.DriveByEncoder(.7, 2, 2, 5);
            }
        }
        else
        {
            if (colorResult == BaseHardware.COLOR_RESULT.BLUE)
            {
                m_coreConfig.Telemetry.addData(">", "In position 3/4, pushing blue ball...");
                m_coreConfig.Telemetry.update();
                m_robot.DriveByEncoder(.7, -2.5, -2.5, 5);
                m_robot.RaiseJewelArm();
                m_robot.DriveByEncoder(.6, 5, 5, 5);
            }
            else if (colorResult == BaseHardware.COLOR_RESULT.RED)
            {
                m_coreConfig.Telemetry.addData(">", "In position 3/4, pushing red ball...");
                m_coreConfig.Telemetry.update();
                m_robot.DriveByEncoder(.7, 2, 2, 5);
                m_robot.RaiseJewelArm();
            }
            else
            {
                m_coreConfig.Telemetry.addData(">", "Raising arm, unknown color...");
                m_coreConfig.Telemetry.update();
                m_robot.RaiseJewelArm();
                m_robot.DriveByEncoder(.7, 2, 2, 5);
            }
        }

        // what is our starting position, run the correct code
        m_coreConfig.Telemetry.update();
        if (m_coreConfig.StartingPosition == CoreConfig.ROBOT_STARTING_POSITION.RED_LEFT_BACK_1)
        {
            ExecuteAutonomousFrom_StartPos_1(glyphColumn);
        }
        else if (m_coreConfig.StartingPosition == CoreConfig.ROBOT_STARTING_POSITION.RED_LEFT_FRONT_2)
        {
            ExecuteAutonomousFrom_StartPos_2(glyphColumn);
        }
        else if (m_coreConfig.StartingPosition == CoreConfig.ROBOT_STARTING_POSITION.BLUE_RIGHT_FRONT_3)
        {
            ExecuteAutonomousFrom_StartPos_3(glyphColumn);
        }
        else if (m_coreConfig.StartingPosition == CoreConfig.ROBOT_STARTING_POSITION.BLUE_RIGHT_BACK_4)
        {
            ExecuteAutonomousFrom_StartPos_4(glyphColumn);
        }

        // open gripper & release glyph
        m_coreConfig.Telemetry.addData(">", "Opening gripper..");
        m_coreConfig.Telemetry.update();
        m_robot.OpenVacuumValve();
        m_robot.liftVacuumMotor.setPower(0);
        m_robot.OpenLiftGripper();

        // back away from block
        m_coreConfig.Telemetry.addData(">", "Backup away from block...");
        m_coreConfig.Telemetry.update();
        m_robot.DriveByEncoder(0.9, -6 , -6, 2);

        // ram the block
        m_robot.DriveByEncoder(0.7, 12, 12, 2);
        m_robot.DriveByEncoder(0.9, -9 , -9, 2);

        // try to get another glyph
        m_robot.RunIntakeRollers();
        if (m_coreConfig.StartingPosition == CoreConfig.ROBOT_STARTING_POSITION.RED_LEFT_BACK_1)
        {
            //ExecuteAutonomousFrom_StartPos_1_Step2();
        }
        else if (m_coreConfig.StartingPosition == CoreConfig.ROBOT_STARTING_POSITION.RED_LEFT_FRONT_2)
        {
            ExecuteAutonomousFrom_StartPos_2_Step2();
        }
        else if (m_coreConfig.StartingPosition == CoreConfig.ROBOT_STARTING_POSITION.BLUE_RIGHT_FRONT_3)
        {
            ExecuteAutonomousFrom_StartPos_2_Step2();
        }
        else if (m_coreConfig.StartingPosition == CoreConfig.ROBOT_STARTING_POSITION.BLUE_RIGHT_BACK_4)
        {
            //ExecuteAutonomousFrom_StartPos_4_Step2();
        }
        m_robot.StopIntakeRollers();
    }

    public void ExecuteAutonomousFrom_StartPos_1(RelicRecoveryVuMark glyphColumn)
    {
        // drive forward x inches
        m_coreConfig.Telemetry.addData(">", "Driving forward...");
        m_coreConfig.Telemetry.update();
        m_robot.DriveByEncoder(.7, 24, 24, 5);

        // turn 90 degrees to the left
        m_coreConfig.Telemetry.addData(">", "Turning left 90 degrees...");
        m_coreConfig.Telemetry.update();
        m_robot.gyroTurn(GYRO_TURN_SPEED, 90, 2);
        m_coreConfig.Telemetry.addData(">", "Holding left 90 degrees...");
        m_coreConfig.Telemetry.update();
        m_robot.gyroHold(GYRO_TURN_SPEED, 90, 2);

        // based on which column we need to target, drive forward the correct number of inches
        if (glyphColumn == RelicRecoveryVuMark.LEFT)
        {
            m_coreConfig.Telemetry.addData(">", "Heading forward to left column...");
            m_coreConfig.Telemetry.update();
            m_robot.DriveByEncoder(0.7, 19, 19, 5);
        }
        else if ((glyphColumn == RelicRecoveryVuMark.CENTER) || (glyphColumn == RelicRecoveryVuMark.UNKNOWN))
        {
            m_coreConfig.Telemetry.addData(">", "Heading forward to middle column...");
            m_coreConfig.Telemetry.update();
            m_robot.DriveByEncoder(0.7, 12, 12, 5);
        }
        else if (glyphColumn == RelicRecoveryVuMark.RIGHT)
        {
            m_coreConfig.Telemetry.addData(">", "Heading forward to right column...");
            m_coreConfig.Telemetry.update();
            m_robot.DriveByEncoder(0.7, 5, 5, 5);
        }

        // turn 90 degrees to the right and face the crypto box
        m_coreConfig.Telemetry.addData(">", "Turning right 90 degrees...");
        m_coreConfig.Telemetry.update();
        m_robot.gyroTurn(GYRO_TURN_SPEED, 0, 2);
        m_coreConfig.Telemetry.addData(">", "Holding right 90 degrees...");
        m_coreConfig.Telemetry.update();
        m_robot.gyroHold(GYRO_TURN_SPEED, 0, 2);

        // drive forward to crypto box x inches
        m_coreConfig.Telemetry.addData(">", "Driving toward crypto box..");
        m_coreConfig.Telemetry.update();
        m_robot.DriveByEncoder(0.8, 12, 12, 5);
    }

    public void ExecuteAutonomousFrom_StartPos_1_Step2()
    {
        // turn a 180 after dropping the glyph
        m_robot.DriveByEncoder(1, -25 , 25, 3);

        // lower the lift
        m_robot.RunLiftToTargetPosition(1, -800, 3);

        // turn the vacuum on
        m_robot.CloseLiftGripper();
        m_robot.CloseVacuumValve();
        m_robot.liftVacuumMotor.setPower(.95);

        // drive forward and hope to connect with a glyph
        m_robot.DriveByEncoder(1, 50 , 50, 3);

        // raise the arm, hopefully with a glyph
        m_robot.RunLiftToTargetPosition(1, 1100, 3);

        m_robot.DriveByEncoder(0.6, -20 , -20, 3);

        m_robot.DriveByEncoder(0.6, -27, 27, 4);

        m_robot.DriveByEncoder(1, 40 , 40, 4);

        m_robot.liftVacuumMotor.setPower(0);
        m_robot.OpenVacuumValve();
        m_robot.OpenLiftGripper();
        m_coreConfig.OpMode.sleep(500);

        m_robot.DriveByEncoder(0.9, -4 , -4, 4);
    }

    public void ExecuteAutonomousFrom_StartPos_2(RelicRecoveryVuMark glyphColumn)
    {
        // based on which column we need to target, drive forward the correct number of inches
        if (glyphColumn == RelicRecoveryVuMark.LEFT)
        {
            m_coreConfig.Telemetry.addData(">", "Heading forward to left column...");
            m_coreConfig.Telemetry.update();
            m_robot.DriveByEncoder(0.7, 43.5, 43.5, 5);
        }
        else if ((glyphColumn == RelicRecoveryVuMark.CENTER) || (glyphColumn == RelicRecoveryVuMark.UNKNOWN))
        {
            m_coreConfig.Telemetry.addData(">", "Heading forward to middle column...");
            m_coreConfig.Telemetry.update();
            m_robot.DriveByEncoder(0.7, 35.5, 35.5, 5);
        }
        else if (glyphColumn == RelicRecoveryVuMark.RIGHT)
        {
            m_coreConfig.Telemetry.addData(">", "Heading forward to right column...");
            m_coreConfig.Telemetry.update();
            m_robot.DriveByEncoder(0.7, 28, 28, 5);
        }

        // turn 90 degrees
        m_coreConfig.Telemetry.addData(">", "Turning left 90 degrees...");
        m_coreConfig.Telemetry.update();
        m_robot.gyroTurn(GYRO_TURN_SPEED, -90, 2);
        m_coreConfig.Telemetry.addData(">", "Holding left 90 degrees...");
        m_coreConfig.Telemetry.update();
        m_robot.gyroHold(GYRO_TURN_SPEED, -90, 2);

        // drive forward to crypto box x inches
        m_coreConfig.Telemetry.addData(">", "Driving toward crypto box..");
        m_coreConfig.Telemetry.update();
        m_robot.DriveByEncoder(0.7, 17 , 17, 5);
    }

    public void ExecuteAutonomousFrom_StartPos_2_Step2()
    {
        // turn a 180 after dropping the glyph
        m_robot.DriveByEncoder(1, -32 , 32, 3);

        // lower the lift
        m_robot.RunLiftToTargetPosition(1, -800, 1);

        // turn the vacuum on
        m_robot.CloseVacuumValve();
        m_robot.liftVacuumMotor.setPower(.95);

        // drive forward and hope to connect with a glyph
        m_robot.DriveByEncoder(1, 45 , 45, 3);

        // raise the arm, hopefully with a glyph
        m_robot.RunLiftToTargetPosition(1, 900, 3);
        m_robot.CloseLiftGripper();

        m_robot.DriveByEncoder(0.7, -20 , -20, 3);

        m_robot.DriveByEncoder(0.7, 29, -29, 4);

        m_robot.DriveByEncoder(1, 35 , 35, 4);

        m_robot.liftVacuumMotor.setPower(0);
        m_robot.OpenVacuumValve();
        m_robot.OpenLiftGripper();
        m_coreConfig.OpMode.sleep(500);

        m_robot.DriveByEncoder(0.9, -6 , -6, 4);
        m_robot.DriveByEncoder(0.9, 6 , 6, 4);
        m_robot.DriveByEncoder(0.9, -6 , -6, 4);
    }

    public void ExecuteAutonomousFrom_StartPos_3(RelicRecoveryVuMark glyphColumn)
    {
        // based on which column we need to target, drive forward the correct number of inches
        if (glyphColumn == RelicRecoveryVuMark.RIGHT)
        {
            m_coreConfig.Telemetry.addData(">", "Heading forward to left column...");
            m_coreConfig.Telemetry.update();
            m_robot.DriveByEncoder(0.7, 43.5, 43.5, 5);
        }
        else if ((glyphColumn == RelicRecoveryVuMark.CENTER) || (glyphColumn == RelicRecoveryVuMark.UNKNOWN))
        {
            m_coreConfig.Telemetry.addData(">", "Heading forward to middle column...");
            m_coreConfig.Telemetry.update();
            m_robot.DriveByEncoder(0.7, 35.5, 35.5, 5);
        }
        else if (glyphColumn == RelicRecoveryVuMark.LEFT)
        {
            m_coreConfig.Telemetry.addData(">", "Heading forward to right column...");
            m_coreConfig.Telemetry.update();
            m_robot.DriveByEncoder(0.7, 28, 28, 5);
        }

        // turn 90 degrees to the left
        m_coreConfig.Telemetry.addData(">", "Turning left 90 degrees...");
        m_coreConfig.Telemetry.update();
        m_robot.gyroTurn(GYRO_TURN_SPEED, 90, 2);
        m_coreConfig.Telemetry.addData(">", "Holding left 90 degrees...");
        m_coreConfig.Telemetry.update();
        m_robot.gyroHold(GYRO_TURN_SPEED, 90, 2);

        // drive forward to crypto box x inches
        m_coreConfig.Telemetry.addData(">", "Driving toward crypto box..");
        m_coreConfig.Telemetry.update();
        m_robot.DriveByEncoder(0.7, 17 , 17, 5);
    }

    public void ExecuteAutonomousFrom_StartPos_4(RelicRecoveryVuMark glyphColumn)
    {

        // drive forward x inches
        m_coreConfig.Telemetry.addData(">", "Driving forward...");
        m_coreConfig.Telemetry.update();
        m_robot.DriveByEncoder(.7, 24, 24, 5);

        // turn 90 degrees to the left
        m_coreConfig.Telemetry.addData(">", "Turning left 90 degrees...");
        m_coreConfig.Telemetry.update();
        m_robot.gyroTurn(GYRO_TURN_SPEED, -90, 2);
        m_coreConfig.Telemetry.addData(">", "Holding left 90 degrees...");
        m_coreConfig.Telemetry.update();
        m_robot.gyroHold(GYRO_TURN_SPEED, -90, 2);

        // based on which column we need to target, drive forward the correct number of inches
        if (glyphColumn == RelicRecoveryVuMark.RIGHT)
        {
            m_coreConfig.Telemetry.addData(">", "Heading forward to left column...");
            m_coreConfig.Telemetry.update();
            m_robot.DriveByEncoder(0.7, 19, 19, 5);
        }
        else if ((glyphColumn == RelicRecoveryVuMark.CENTER) || (glyphColumn == RelicRecoveryVuMark.UNKNOWN))
        {
            m_coreConfig.Telemetry.addData(">", "Heading forward to middle column...");
            m_coreConfig.Telemetry.update();
            m_robot.DriveByEncoder(0.7, 11, 11, 5);
        }
        else if (glyphColumn == RelicRecoveryVuMark.LEFT)
        {
            m_coreConfig.Telemetry.addData(">", "Heading forward to right column...");
            m_coreConfig.Telemetry.update();
            m_robot.DriveByEncoder(0.7, 5, 5, 5);
        }

        // turn 90 degrees to the right and face the crypto box
        m_coreConfig.Telemetry.addData(">", "Turning right 90 degrees...");
        m_coreConfig.Telemetry.update();
        m_robot.gyroTurn(GYRO_TURN_SPEED, 0, 2);
        m_coreConfig.Telemetry.addData(">", "Holding right 90 degrees...");
        m_coreConfig.Telemetry.update();
        m_robot.gyroHold(GYRO_TURN_SPEED, 0, 2);

        // drive forward to crypto box x inches
        m_coreConfig.Telemetry.addData(">", "Driving toward crypto box..");
        m_coreConfig.Telemetry.update();
        m_robot.DriveByEncoder(0.7, 12, 12, 5);
    }

    public void ExecuteAutonomousFrom_StartPos_4_Step2()
    {
        // turn a 180 after dropping the glyph
        m_robot.DriveByEncoder(1, 25 , -25, 3);

        // lower the lift
        m_robot.RunLiftToTargetPosition(1, -800, 3);

        // turn the vacuum on
        m_robot.CloseLiftGripper();
        m_robot.CloseVacuumValve();
        m_robot.liftVacuumMotor.setPower(.95);

        // drive forward and hope to connect with a glyph
        m_robot.DriveByEncoder(1, 50 , 50, 3);

        // raise the arm, hopefully with a glyph
        m_robot.RunLiftToTargetPosition(1, 1100, 3);

        m_robot.DriveByEncoder(0.6, -20 , -20, 3);

        m_robot.DriveByEncoder(0.6, -27, 27, 4);

        m_robot.DriveByEncoder(1, 40 , 40, 4);

        m_robot.liftVacuumMotor.setPower(0);
        m_robot.OpenVacuumValve();
        m_robot.OpenLiftGripper();
        m_coreConfig.OpMode.sleep(500);

        m_robot.DriveByEncoder(0.9, -4 , -4, 4);
    }
}
