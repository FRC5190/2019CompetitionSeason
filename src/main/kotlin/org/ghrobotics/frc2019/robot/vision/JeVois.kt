@file:Suppress("MatchingDeclarationName")

package org.ghrobotics.frc2019.robot.vision

import com.github.salomonbrys.kotson.fromJson
import com.google.gson.Gson
import com.google.gson.JsonObject
import com.google.gson.JsonParseException
import edu.wpi.first.wpilibj.SerialPort
import edu.wpi.first.wpilibj.Timer
import kotlinx.coroutines.channels.SendChannel
import kotlinx.coroutines.channels.sendBlocking
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d
import org.ghrobotics.lib.mathematics.units.*
import java.util.concurrent.TimeUnit
import kotlin.concurrent.thread
import kotlin.math.pow

class JeVois(
    private val port: SerialPort.Port,
    private val visionDataChannel: SendChannel<VisionData>
) {

    private var fpgaOffset = 0.second

    init {
        thread {
            while (true) {
                try {
                    val port = createSerialPort()
                    initPort(port)
                    readPort(port)
                } catch (@Suppress("TooGenericExceptionCaught") e: Throwable) {
                    e.printStackTrace()
                    println("[JeVois-${port.name}] Failure in connection!!!")
                }
            }
        }
    }

    private fun createSerialPort(): SerialPort {
        println("[JeVois-${port.name}] Trying to create serial port...")
        while (true) {
            try {
                val serialPort = SerialPort(115200, port)
                serialPort.setTimeout(0.5)
                return serialPort
            } catch (@Suppress("TooGenericExceptionCaught") e: Throwable) {
                e.printStackTrace()
                TimeUnit.MILLISECONDS.sleep(10)
            }
        }
    }

    private fun initPort(serialPort: SerialPort) {
        serialPort.writeString("setpar serout USB\n")

        serialPort.writeString("date 0101000070\n")
        fpgaOffset = Timer.getFPGATimestamp().second
    }

    private fun readPort(serialPort: SerialPort) {
        while (true) {
            val line = serialPort.readLine()

            if (!line.startsWith('{')) continue
            try {
                val jsonData = kJevoisGson.fromJson<JsonObject>(line)

                val timestamp = (jsonData["Epoch Time"].asDouble.second + fpgaOffset)
                val contours = jsonData["Targets"].asJsonArray
                    .asSequence()
                    .filterIsInstance<JsonObject>()
                    .map { contourData ->
                        VisionTarget(
                            -contourData["angle"].asDouble.degree,
                            contourData["distance"].asDouble.inch
                        )
                    }.toList()

                visionDataChannel.sendBlocking(VisionData(timestamp, contours))
            } catch (e: JsonParseException) {
                e.printStackTrace()
                println("[JeVois-${port.name}] Got Invalid Data: $line")
            }
        }
    }

    private val tempStringBuilder = StringBuilder()
    private fun SerialPort.readLine(): String {
        tempStringBuilder.clear()
        while (true) {
            val nextChar = read(1).first().toChar()
            if (nextChar == '\n') {
                val result = tempStringBuilder.toString()
                tempStringBuilder.clear()
                return result
            }
            tempStringBuilder.append(nextChar)
        }
    }

    companion object {
        private val kJevoisGson = Gson()
    }

}

data class VisionData(
    val timestamp: Time,
    val targets: List<VisionTarget>
)

data class VisionTarget(
    val angle: Rotation2d,
    val distance: Length
) {
    val cameraRelativePose: Pose2d

    init {
        val a = 5.0.inch.meter.pow(2)
        val c = distance.meter.pow(2)
        val b = Math.sqrt(c - a).meter

        cameraRelativePose = Pose2d(Translation2d(b, angle), 0.degree)
    }
}