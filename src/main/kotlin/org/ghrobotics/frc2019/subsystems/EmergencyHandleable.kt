package org.ghrobotics.frc2019.subsystems

import org.ghrobotics.frc2019.Robot

interface EmergencyHandleable {
    fun activateEmergency()
    fun recoverFromEmergency()
}