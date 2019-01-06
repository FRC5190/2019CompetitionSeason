/*
 * FRC Team 5190
 * Green Hope Falcons
 */

package org.ghrobotics.frc2019.robot

import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.units.degree
import org.ghrobotics.lib.mathematics.units.feet
import org.ghrobotics.lib.mathematics.units.inch
import org.ghrobotics.lib.mathematics.units.meter
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
    const val kLeftSlaveId2 = 5

    const val kRightMasterId = 3
    const val kRightSlaveId1 = 4
    const val kRightSlaveId2 = 6

    // GYROS
    const val kPigeonIMUId = 17

    // PNEUMATICS
    const val kPCMId = 41
    const val kDriveSolenoidId = 3 // TODO Find Actual Value

    // FIELD
    val kLevel2RightX = 4.feet
    val kLevel2BottomY = 97.inch

    val kHypotenuseDifferenceForRamp = 0.433.inch

    // ROBOT
    val kRobotWidth = 27.inch // TODO Find Actual Value
    val kRobotLength = 33.inch // TODO Find Actual Value
    val kBumperLength = 2.0.inch // TODO Find Actual Value

    const val kRobotMass = 60.0 // kg // TODO Find Actual Value
    const val kRobotMomentOfInertia = 10.0 // kg m^2 // TODO Find Actual Value
    const val kRobotAngularDrag = 12.0 // N*m / (rad/sec) // TODO Find Actual Value

    // TRANSFORMATIONS
    val kCenterToFrontBumper = Pose2d(-(kRobotLength / 2.0) - kBumperLength, 0.meter, 0.degree)

    // DRIVE
    val kDriveSensorUnitsPerRotation = 1440.STU // TODO Find Actual Value
    val kWheelRadius = 2.92.inch // TODO Find Actual Value
    val kTrackWidth = 0.8128.meter // TODO Find Actual Value

    val kDriveNativeUnitModel = NativeUnitLengthModel(
        kDriveSensorUnitsPerRotation,
        kWheelRadius
    )

    const val kPDrive = 1.7 // Talon SRX Units
    const val kDDrive = 10.0

    const val kStaticFrictionVoltage = 1.2 // Volts // TODO Find Actual Value
    const val kVDrive = 0.185 // Volts per radians per second // TODO Find Actual Value
    const val kADrive = 0.020 // Volts per radians per second per second // TODO Find Actual Value

    const val kDriveBeta = 2.00 // Inverse meters squared // TODO Find Actual Value
    const val kDriveZeta = 0.90 // Unitless dampening co-efficient // TODO Find Actual Value
}
