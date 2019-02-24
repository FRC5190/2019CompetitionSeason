package org.ghrobotics.frc2019.subsystems.climb

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.GenericHID
import org.ghrobotics.frc2019.Controls
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.wrappers.hid.getY
import kotlin.math.absoluteValue

class ManualClimbCommand : FalconCommand(ClimbSubsystem) {

    var set = false

    override suspend fun execute() {
        if (Controls.isClimbing) {

            val frontWinch = -frontWinchSource()
            val backWinch = -backWinchSource()
            val wheelPercent = wheelSource()

            if (frontWinch.absoluteValue > 0.1 || backWinch.absoluteValue > 0.2 || wheelPercent.absoluteValue > 0.2) {
                ClimbSubsystem.frontWinchPercentOutput = frontWinch
                ClimbSubsystem.backWinchPercentOutput = backWinch
                ClimbSubsystem.wheelPercentOutput = wheelPercent
                set = false
            } else if (!set) {
                ClimbSubsystem.frontWinchMaster.set(ControlMode.MotionMagic, ClimbSubsystem.rawFront.toDouble())
                ClimbSubsystem.backWinchMaster.set(ControlMode.MotionMagic, ClimbSubsystem.rawBack.toDouble())
                set = true
            }
        } else {
            ClimbSubsystem.frontWinchPercentOutput = 0.0
            ClimbSubsystem.backWinchPercentOutput = 0.0
            ClimbSubsystem.wheelPercentOutput = 0.0
        }
    }

    private companion object {
        val frontWinchSource = Controls.operatorXbox.getY(GenericHID.Hand.kLeft)
        val backWinchSource = Controls.operatorXbox.getY(GenericHID.Hand.kRight)
        val wheelSource = Controls.driverXbox.getY(GenericHID.Hand.kRight)
    }
}
