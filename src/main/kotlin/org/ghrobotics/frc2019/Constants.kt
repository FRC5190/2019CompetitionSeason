/*
 * FRC Team 5190
 * Green Hope Falcons
 */

package org.ghrobotics.frc2019

import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Rectangle2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d
import org.ghrobotics.lib.mathematics.units.*
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnitLengthModel
import org.ghrobotics.lib.mathematics.units.nativeunits.STU

@Suppress("MemberVisibilityCanBePrivate", "unused")
object Constants {

    /* TODO ALL CONSTANTS MARKED WITH THE FLAG ARE FALCON HEAVY 2018'S CONSTANTS */

    // GLOBAL CTRE TIMEOUT
    const val kCTRETimeout = 10

    // MOTOR IDS
    const val kLeftMasterId = 1
    const val kLeftSlaveId1 = 2

    const val kRightMasterId = 3
    const val kRightSlaveId1 = 4

    const val kElevatorMasterId = 5
    const val kElevatorSlave1Id = 6
    const val kElevatorSlave2Id = 7
    const val kElevatorSlave3Id = 8

    // GYROS
    const val kPigeonIMUId = 17

    // PNEUMATICS
    const val kPCMId = 41
    const val kDriveSolenoidId = 3

    // FIELD
    val kLevel2RightX = 4.feet
    val kLevel2BottomY = 8.feet
    val kLevel1RightX = 7.feet
    val kLevel1Platform = Rectangle2d(Translation2d(4.feet, 7.feet), Translation2d(8.feet, 20.feet))

    val kHypotenuseDifferenceForRamp = 0.433.inch

    // ROBOT
    val kRobotWidth = 29.inch
    val kRobotLength = 30.inch
    val kBumperLength = 4.5.inch
    val kIntakeLength = 10.inch


    const val kRobotMass = 18.0 // kg
    const val kRobotMomentOfInertia = 9.0 // kg m^2
    const val kRobotAngularDrag = 6.0

    // TRANSFORMATIONS
    val kFrontBumperToCenter = Pose2d(-(kRobotLength / 2.0) - kBumperLength, 0.meter, 0.degree)
    val kForwardIntakeToCenter = Pose2d(-(kRobotLength / 2.0) - kIntakeLength, 0.meter, 0.degree)
    val kBackwardIntakeToCenter = Pose2d(kRobotLength / 2.0 + kIntakeLength, 0.meter, 0.degree)
    val kCenterToCamera = Pose2d(7.inch, (-5.0).inch, 0.degree)
    val kGroundToCamera = 0.meter

    val kMaxTargetTrackingDistance = 2.feet
    val kMaxTargetTrackingLifetime = 1.second

    // DRIVE
    val kDriveSensorUnitsPerRotation = 1440.STU
    val kWheelRadius = 3.0856025633758268193941419970783.inch
    val kTrackWidth = 27.75.inch

    val kDriveNativeUnitModel = NativeUnitLengthModel(
        kDriveSensorUnitsPerRotation,
        kWheelRadius
    )

    val kDriveCurrentLimit = 40.amp

    const val kDriveKp = 1.2 // Talon SRX Units
    const val kDriveKd = 0.0

    const val kStaticFrictionVoltage = 0.8 // Volts
    const val kDriveKv = 0.135 // Volts per radians per second // TODO Find Actual Value
    const val kDriveKa = 0.012 // Volts per radians per second per second // TODO Find Actual Value

    const val kDriveBeta = 2.0 // Inverse meters squared
    const val kDriveZeta = 0.7 // Unitless dampening co-efficient


    // ELEVATOR
    val kElevatorSensorUnitsPerRotaton = 1440.STU
    val kElevatorWinchRadius = 1.25.inch
    val kElevatorNativeUnitModel = NativeUnitLengthModel(
        kDriveSensorUnitsPerRotation,
        kElevatorWinchRadius
    )

    val kElevatorCurrentLimit = 15.amp

    val kElevatorClosedLoopTolerance = 1.inch

    const val kElevatorKp = 0.3 // TODO Tune me
    const val kElevatorKg = 0.05 // Percent required to counteract gravity // TODO Tune me
}
