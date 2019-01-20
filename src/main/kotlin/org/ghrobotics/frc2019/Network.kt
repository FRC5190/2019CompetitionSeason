/*
 * FRC Team 5190
 * Green Hope Falcons
 */

package org.ghrobotics.frc2019

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab
import org.ghrobotics.frc2019.auto.AutoMode
import org.ghrobotics.frc2019.auto.StartingPositions
import org.ghrobotics.frc2019.subsystems.arm.ArmSubsystem
import org.ghrobotics.frc2019.subsystems.drive.DriveSubsystem
import org.ghrobotics.frc2019.subsystems.elevator.ElevatorSubsystem
import org.ghrobotics.frc2019.vision.TargetTracker
import org.ghrobotics.lib.mathematics.units.derivedunits.inchesPerSecond
import org.ghrobotics.lib.wrappers.networktables.enumSendableChooser

object Network {

    val startingPositionChooser = enumSendableChooser<StartingPositions>()
    val autoModeChooser = enumSendableChooser<AutoMode>()

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

    private val globalXEntry = localizationLayout.add("Robot X", 0.0).entry
    private val globalYEntry = localizationLayout.add("Robot Y", 0.0).entry
    private val globalAEntry = localizationLayout.add("Robot Angle", 0.0).entry

    private val leftPositionEntry = driveSubsystemLayout.add("Left Encoder", 0.0).entry
    private val rightPositionEntry = driveSubsystemLayout.add("Right Encoder", 0.0).entry
    private val leftAmperageEntry = driveSubsystemLayout.add("Left Current", 0.0).entry
    private val rightAmperageEntry = driveSubsystemLayout.add("Right Current", 0.0).entry

    private val visionTargetX = visionLayout.add("Vision Target X", 0.0).entry
    val visionLastUpdate = visionLayout.add("Vision Last Update", 0.0).entry
    private val visionTargetY = visionLayout.add("Vision Target Y", 0.0).entry
    private val visionTargetRotation = visionLayout.add("Vision Target Rotation", 0.0).entry

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

    val visionDriveAngle = visionLayout.add("Vision Drive Angle", 0.0).entry
    val visionDriveActive = visionLayout.add("Vision Drive Active", false).entry

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

    fun update() {
        globalXEntry.setDouble(DriveSubsystem.localization().translation.x.feet)
        globalYEntry.setDouble(DriveSubsystem.localization().translation.y.feet)
        globalAEntry.setDouble(DriveSubsystem.localization().rotation.degree)

        leftPositionEntry.setDouble(DriveSubsystem.leftMotor.getSelectedSensorPosition(0).toDouble())
        rightPositionEntry.setDouble(DriveSubsystem.rightMotor.getSelectedSensorPosition(0).toDouble())

        leftAmperageEntry.setDouble(DriveSubsystem.leftMotor.outputCurrent)
        rightAmperageEntry.setDouble(DriveSubsystem.rightMotor.outputCurrent)

        elevatorRawPosition.setDouble(ElevatorSubsystem.rawEncoder.toDouble())
        elevatorPosition.setDouble(ElevatorSubsystem.elevatorPosition.inch)
        elevatorCurrent.setDouble(ElevatorSubsystem.current)
        elevatorVoltage.setDouble(ElevatorSubsystem.voltage)
//        elevatorVelocity.setDouble(ElevatorSubsystem.velocity.inchesPerSecond)
        elevatorLimitSwitch.setBoolean(ElevatorSubsystem.limitSwitch)

        armRawPosition.setDouble(ArmSubsystem.rawEncoder.toDouble())
        armPosition.setDouble(ArmSubsystem.armPosition.degree)
        armCurrent.setDouble(ArmSubsystem.current)
        armVoltage.setDouble(ArmSubsystem.voltage)
        armVelocity.setDouble(ArmSubsystem.velocity.value * 180 / Math.PI)

        val trackedObject = TargetTracker.bestTarget
        if (trackedObject != null) {
            val visionTargetPose = trackedObject.averagePose
            visionTargetX.setDouble(visionTargetPose.translation.x.inch)
            visionTargetY.setDouble(visionTargetPose.translation.y.inch)
            visionTargetRotation.setDouble(visionTargetPose.rotation.degree)
        }
    }
}