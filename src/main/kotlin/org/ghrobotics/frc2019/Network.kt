/*
 * FRC Team 5190
 * Green Hope Falcons
 */

package org.ghrobotics.frc2019

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab
import org.ghrobotics.frc2019.auto.Autonomous
import org.ghrobotics.frc2019.subsystems.arm.ArmSubsystem
import org.ghrobotics.frc2019.subsystems.climb.ClimbSubsystem
import org.ghrobotics.frc2019.subsystems.drive.DriveSubsystem
import org.ghrobotics.frc2019.subsystems.elevator.ElevatorSubsystem
import org.ghrobotics.frc2019.subsystems.intake.IntakeSubsystem
import org.ghrobotics.frc2019.vision.JeVoisManager
import org.ghrobotics.lib.mathematics.units.SILengthConstants
import org.ghrobotics.lib.wrappers.networktables.enumSendableChooser

object Network {

    val startingPositionChooser = enumSendableChooser<Autonomous.StartingPositions>()
    val autoModeChooser = enumSendableChooser<Autonomous.Mode>()

    private val mainShuffleboardDisplay: ShuffleboardTab = Shuffleboard.getTab("5190")

    private val autoLayout = mainShuffleboardDisplay.getLayout("Autonomous", BuiltInLayouts.kList)
        .withSize(2, 2)
        .withPosition(0, 0)

    private val localizationLayout = mainShuffleboardDisplay.getLayout("Localization", BuiltInLayouts.kList)
        .withSize(2, 2)
        .withPosition(2, 0)

    private val visionLayout = mainShuffleboardDisplay.getLayout("Vision", BuiltInLayouts.kGrid)
        .withSize(3, 3)
        .withPosition(0, 2)

    private val driveSubsystemLayout = mainShuffleboardDisplay.getLayout("Drive", BuiltInLayouts.kGrid)
        .withSize(2, 2)
        .withPosition(4, 0)

    private val elevatorSubsystemLayout = mainShuffleboardDisplay.getLayout("Elevator", BuiltInLayouts.kGrid)
        .withSize(2, 2)
        .withPosition(6, 0)

    private val armSubsystemLayout = mainShuffleboardDisplay.getLayout("Arm", BuiltInLayouts.kGrid)
        .withSize(2, 2)
        .withPosition(8, 0)

    private val climbSubsystemLayout = mainShuffleboardDisplay.getLayout("Climb", BuiltInLayouts.kGrid)
        .withSize(3, 2)
        .withPosition(10, 0)

    private val globalXEntry = localizationLayout.add("Robot X", 0.0).entry
    private val globalYEntry = localizationLayout.add("Robot Y", 0.0).entry
    private val globalAEntry = localizationLayout.add("Robot Angle", 0.0).entry

    private val leftPositionEntry = driveSubsystemLayout.add("Left Encoder", 0.0).entry
    private val rightPositionEntry = driveSubsystemLayout.add("Right Encoder", 0.0).entry
    private val leftAmperageEntry = driveSubsystemLayout.add("Left Current", 0.0).entry
    private val rightAmperageEntry = driveSubsystemLayout.add("Right Current", 0.0).entry

    private val visionFrontCameraConnected = visionLayout.add("Front Camera Connected", false).entry
    private val visionBackCameraConnected = visionLayout.add("Back Camera Connected", false).entry
//    private val visionTargetX = visionLayout.add("Vision Target X", 0.0).entry
//    val visionLastUpdate = visionLayout.add("Vision Last Update", 0.0).entry
//    private val visionTargetY = visionLayout.add("Vision Target Y", 0.0).entry
//    private val visionTargetRotation = visionLayout.add("Vision Target Rotation", 0.0).entry

    private val elevatorRawPosition = elevatorSubsystemLayout.add("Raw Position", 0.0).entry
    private val elevatorPosition = elevatorSubsystemLayout.add("Position (in)", 0.0).entry
    private val elevatorCurrent = elevatorSubsystemLayout.add("Current", 0.0).entry
    private val elevatorVoltage = elevatorSubsystemLayout.add("Voltage", 0.0).entry
    private val elevatorVelocity = elevatorSubsystemLayout.add("Velocity (ips)", 0.0).entry
    private val elevatorLimitSwitch = elevatorSubsystemLayout.add("Rev Limit Switch", false).entry

    private val armRawPosition = armSubsystemLayout.add("Raw Position", 0.0).entry
    private val armPosition = armSubsystemLayout.add("Position (deg)", 0.0).entry
    private val armCurrent = armSubsystemLayout.add("Current", 0.0).entry
    private val armVoltage = armSubsystemLayout.add("Voltage", 0.0).entry
    private val armVelocity = armSubsystemLayout.add("Velocity (dps)", 0.0).entry
    private val isHoldingCargo = armSubsystemLayout.add("Intake Holding Cargo", false).entry
    private val intakeFullyExtended = armSubsystemLayout.add("Intake Fully Extended", false).entry

    private val frontClimbWinchPosition = climbSubsystemLayout.add("Front Winch Position (in)", 0.0).entry
    private val backClimbWinchPosition = climbSubsystemLayout.add("Back Winch Position (in)", 0.0).entry
    private val climbLidarRaw = climbSubsystemLayout.add("Lidar Raw", 0.0).entry
    private val frontClimbWinchCurrent = climbSubsystemLayout.add("Front Winch Current", 0.0).entry
    private val backClimbWinchCurrent = climbSubsystemLayout.add("Back Winch Current", 0.0).entry
    private val frontLimitSwitch = climbSubsystemLayout.add("F Limit", false).entry
    private val backLimitSwitch = climbSubsystemLayout.add("R Limit", false).entry

    val visionDriveAngle = visionLayout.add("Vision Drive Angle", 0.0).entry
    val visionDriveActive = visionLayout.add("Vision Drive Active", false).entry
    val badIntakeOffset = visionLayout.add("Intake Offset", IntakeSubsystem.badIntakeOffset.inch).entry

    private var debugEnabled = mainShuffleboardDisplay.add("Debug Active", Robot.debugActive).entry

    init {
        // Put choosers on dashboard
        autoLayout.add(
            "Auto Mode",
            autoModeChooser
        )
        autoLayout.add(
            "Starting Position",
            startingPositionChooser
        )

        //mainShuffleboardDisplay.add(VisionProcessing.cameraSource).withPosition(3, 2).withSize(3, 3)
    }

    @Suppress("LongMethod")
    fun update() {
        globalXEntry.setDouble(DriveSubsystem.localization().translation.x / SILengthConstants.kFeetToMeter)
        globalYEntry.setDouble(DriveSubsystem.localization().translation.y / SILengthConstants.kFeetToMeter)
        globalAEntry.setDouble(DriveSubsystem.localization().rotation.degree)

        leftPositionEntry.setDouble(DriveSubsystem.leftMotor.getSelectedSensorPosition(0).toDouble())
        rightPositionEntry.setDouble(DriveSubsystem.rightMotor.getSelectedSensorPosition(0).toDouble())

        leftAmperageEntry.setDouble(DriveSubsystem.leftMotor.outputCurrent)
        rightAmperageEntry.setDouble(DriveSubsystem.rightMotor.outputCurrent)

        elevatorRawPosition.setDouble(ElevatorSubsystem.rawSensorPosition.toDouble())
        elevatorPosition.setDouble(ElevatorSubsystem.position / SILengthConstants.kInchToMeter)
        elevatorCurrent.setDouble(ElevatorSubsystem.current)
        elevatorVoltage.setDouble(ElevatorSubsystem.voltage)
        elevatorVelocity.setDouble(ElevatorSubsystem.velocity / SILengthConstants.kInchToMeter)
        elevatorLimitSwitch.setBoolean(ElevatorSubsystem.isBottomLimitSwitchPressed)

        armRawPosition.setDouble(ArmSubsystem.rawSensorPosition.toDouble())
        armPosition.setDouble(ArmSubsystem.position.degree)
        armCurrent.setDouble(ArmSubsystem.current)
        armVoltage.setDouble(ArmSubsystem.voltage)
        armVelocity.setDouble(ArmSubsystem.velocity.value * 180 / Math.PI)

        isHoldingCargo.setBoolean(IntakeSubsystem.isSeeingCargo)
//        intakeFullyExtended.setBoolean(IntakeSubsystem.isFullyExtended)

        frontClimbWinchPosition.setDouble(ClimbSubsystem.rawFrontWinchPosition.toDouble())
        backClimbWinchPosition.setDouble(ClimbSubsystem.rawBackWinchPosition.toDouble())
        climbLidarRaw.setDouble(ClimbSubsystem.lidarRawAveraged)
        frontClimbWinchCurrent.setDouble(ClimbSubsystem.frontWinchCurrent)
        backClimbWinchCurrent.setDouble(ClimbSubsystem.backWinchCurrent)
        frontLimitSwitch.setBoolean(ClimbSubsystem.isFrontReverseLimitSwitchClosed)
        backLimitSwitch.setBoolean(ClimbSubsystem.isBackReverseLimitSwitchClosed)

        visionFrontCameraConnected.setBoolean(JeVoisManager.isFrontJeVoisConnected)
        visionBackCameraConnected.setBoolean(JeVoisManager.isBackJeVoisConnected)

        visionFrontCameraConnected.setBoolean(JeVoisManager.isFrontJeVoisConnected)
        visionBackCameraConnected.setBoolean(JeVoisManager.isBackJeVoisConnected)

        badIntakeOffset.setDouble(IntakeSubsystem.badIntakeOffset.inch)

        Robot.debugActive = debugEnabled.getBoolean(Robot.debugActive)

//        val trackedObject = TargetTracker.bestTarget
//        if (trackedObject != null) {
//            val visionTargetPose = trackedObject.averagePose
//            visionTargetX.setDouble(visionTargetPose.translation.x.inch)
//            visionTargetY.setDouble(visionTargetPose.translation.y.inch)
//            visionTargetRotation.setDouble(visionTargetPose.rotation.degree)
//        }
    }
}