package org.ghrobotics.frc2019.subsystems.climb

import org.ghrobotics.frc2019.subsystems.drive.DriveSubsystem
import org.ghrobotics.lib.commands.*
import org.ghrobotics.lib.mathematics.units.inch
import org.ghrobotics.lib.mathematics.units.second
import org.ghrobotics.lib.utils.DoubleSource

object NewAutoClimbRoutines {

    private var aboveLIDARThresholdHeight = false

    fun autoClimb(isLevel2: Boolean) = sequential {
        // Reset LIDAR Threshold
        +InstantRunnableCommand { aboveLIDARThresholdHeight = false }

        // Step 1: Extend stilts and begin moving forward toward the HAB platform.
        +parallel {
            // Climb to height
            if (isLevel2) {
                +ClosedLoopClimbCommand(frontTarget = 8.inch, backTarget = 9.inch)
            } else {
                +ClosedLoopClimbCommand(frontTarget = 23.5.inch, backTarget = 18.5.inch)
            }

            // Start moving forward
            +sequential {
                if (isLevel2) {
                    +ConditionCommand {
                        ClimbSubsystem.frontWinchPosition > 6.5.inch.value &&
                            ClimbSubsystem.backWinchPosition > 7.5.inch.value
                    }
                } else {
                    +ConditionCommand {
                        ClimbSubsystem.frontWinchPosition > 19.inch.value &&
                            ClimbSubsystem.backWinchPosition > 15.inch.value
                    }
                }
                +InstantRunnableCommand { aboveLIDARThresholdHeight = true }
                +ClimbWheelCommand { 0.75 }
            }
        }.withExit {
            // Exit only when LIDAR is detected, robot is above platform, and the stilts are above a certain
            // threshold height.
            ClimbSubsystem.lidarRawAveraged > 25 && ClimbSubsystem.lidarRawAveraged < 600 &&
                aboveLIDARThresholdHeight
        }

        // Step 2: Retract back stilts
        +parallel {
            +ResetWinchCommand(resetFront = false)
            +ClimbWheelCommand { 0.5 }
            +DriveWithPercentCommand { -0.05 }
        }.withExit { ClimbSubsystem.isBackReverseLimitSwitchClosed && ClimbSubsystem.rawBackWinchPosition < 2000 }

        // Step 3: Drive until the front is safe to retract
        +parallel {
            +DriveWithPercentCommand { -0.4 }
            +ClimbWheelCommand { 1.0 }
        }.apply {
            if (isLevel2) {
                withTimeout(1.75.second)
            } else {
                withExit { ClimbSubsystem.frontOnPlatform }
            }
        }

        // Step 4: Retract front stilts
        +parallel {
            +ResetWinchCommand(resetFront = true)
            +DriveWithPercentCommand { -0.05 }
        }.withExit { ClimbSubsystem.isFrontReverseLimitSwitchClosed && ClimbSubsystem.rawFrontWinchPosition < 2000 }

        // Step 5: Drive
        +DriveWithPercentCommand { -0.5 }
    }


    class DriveWithPercentCommand(val percentSource: DoubleSource) : FalconCommand(DriveSubsystem) {
        override suspend fun execute() {
            DriveSubsystem.tankDrive(percentSource(), percentSource())
        }

        override suspend fun dispose() {
            DriveSubsystem.zeroOutputs()
        }
    }
}