import chisel3._
import org.scalatest._
import chiseltest._
import chiseltest.ChiselScalatestTester
import chiseltest.internal.VerilatorBackendAnnotation
import chiseltest.experimental.TestOptionBuilder._
import org.scalatest.FreeSpec
import scala.io.Source

class TopTest extends FreeSpec with ChiselScalatestTester {
  def getFile: Option[String] = {
    if (scalaTestContext.value.get.configMap.contains("memFile")) {
      Some(scalaTestContext.value.get.configMap("memFile").toString)
    } else {
      None
    }
  }
  "should just work" in {
    val programFile = getFile
    test(new Top(programFile)).withAnnotations(Seq(VerilatorBackendAnnotation)) {c =>
      c.clock.setTimeout(0)
      c.clock.step(8000)
    }
  }
}


///////////////////////

// def getFile: Option[String] = {
//     if (scalaTestContext.value.get.configMap.contains("memFile")) {
//       Some(scalaTestContext.value.get.configMap("memFile").toString)
//     } else {
//       None
//     }
//   }
//   "should just work" in {
//     val programFile = getFile
//     test(new Top(None)).withAnnotations(Seq(VerilatorBackendAnnotation)) {c =>
//     // c.io.CLK_PER_BIT.poke(4.U)


//     val bufferedSource =  Source.fromFile(programFile.get)
//     val fileData = bufferedSource.getLines.toArray
//     val insts = for (i <- fileData) yield java.lang.Long.parseLong(i.substring(2), 16)
//     bufferedSource.close

//     c.io.rx_i.poke(1.U)


//     c.clock.step(10)

//   // val instss = Array(0x00400293, 0x00400313, 0x006283B3, 0x00000fff)
//     for (inst <- insts) {
//         // val inst = 0xf0f0f0f0
//         val half_byte1 = inst & 0x0f // 3
//         val half_byte2 = (inst & 0xf0) >> 4 // 1
//         val byte1 = (half_byte2 << 4) | half_byte1 // 0x13

//         val half_byte3 = (inst & 0xf00) >> 8  // 1
//         val half_byte4 = (inst & 0xf000) >> 12  // 0
//         val byte2 = (half_byte4 << 4) | half_byte3  // 0x01

//         val half_byte5 = (inst & 0xf0000) >> 16 // 0
//         val half_byte6 = (inst & 0xf00000) >> 20  // 2
//         val byte3 = (half_byte6 << 4) | half_byte5  // 0x20

//         val half_byte7 = (inst & 0xf000000) >> 24 // 0
//         val half_byte8 = (inst & 0xf0000000) >> 28  // 0
//         val byte4 = (half_byte8 << 4) | half_byte7  // 0x00

//         //printf("The instruction is %x".format(byte4))
//         pokeUart(byte1.toInt)
//         pokeUart(byte2.toInt)
//         pokeUart(byte3.toInt)
//         pokeUart(byte4.toInt)
//     }

//         def pokeUart(value: Int): Unit = {

//             // start bit
//             // poke(c.io.rx_i, 0)
//             c.io.rx_i.poke(0.U)
//             // step(4)
//             c.clock.step(4)
//             // 8 data bits
//             for (i <- 0 until 8) {
//             // poke(c.io.rx_i, (value >> i) & 0x01)
//             c.io.rx_i.poke(((value >> i) & 0x01).U)
//             // step(4)
//             c.clock.step(4)
//             }
//             // stop bit
//             // poke(c.io.rx_i, 1)
//             c.io.rx_i.poke(1.U)
//             // step(4)
//             c.clock.step(4)
//         }
//       c.clock.step(200)
//       c.clock.setTimeout(0)
//       c.clock.step(1000)
//     }
//   }
// }