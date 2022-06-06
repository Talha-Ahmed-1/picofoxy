import chisel3._
// import buraq_mini.core.Core
import nucleusrv.components.Core
import caravan.bus.common.{AddressMap, BusDecoder, Switch1toN}
import caravan.bus.wishbone.{Peripherals, WBRequest, WBResponse, WishboneConfig, WishboneDevice, WishboneErr, WishboneHost, WishboneMaster, WishboneSlave}
import chisel3.experimental.Analog
import chisel3.stage.ChiselStage
import jigsaw.fpga.boards.artyA7._
import jigsaw.rams.fpga.BlockRam
import jigsaw.peripherals.gpio._
import chisel3.util.Enum
import programmable_uart._

class Picofoxy(programFile: Option[String]) extends Module {
  val io = IO(new Bundle {
    val gpio_io = Vec(4, Analog(1.W))
    val rx_i = Input(UInt(1.W))
    val done = Output(Bool())
  })

  val top = Module(new Top(programFile))
  val pll = Module(new PLL_8MHz())

  pll.io.clk_in1 := clock
  top.clock := pll.io.clk_out1

  val gpioInputWires = Wire(Vec(4, Bool()))
  val gpioOutputWires = Wire(Vec(4, Bool()))
  val gpioEnableWires = Wire(Vec(4, Bool()))

  val gpioPads = TriStateBuffer(quantity=4)
  val triStateBufferWires = for {
    ((((a,b),c),d),e) <- gpioPads zip gpioInputWires zip gpioOutputWires zip gpioEnableWires zip io.gpio_io
  } yield (a,b,c,d,e)

  triStateBufferWires map { case(buf: IOBUF, in: Bool, out: Bool, en: Bool, io: Analog) => {
    buf.io.connect(in, out, io, en)
  }}

  top.io.gpio_i := gpioInputWires.asUInt()
  gpioOutputWires := top.io.gpio_o.asBools()
  gpioEnableWires := top.io.gpio_en_o.asBools()

  top.io.rx_i := io.rx_i
  io.done := top.io.done
}


class Top(programFile: Option[String]) extends Module {
  val io = IO(new Bundle {
    val gpio_o = Output(UInt(4.W))
    val gpio_en_o = Output(UInt(4.W))
    val gpio_i = Input(UInt(4.W))

    val rx_i = Input(UInt(1.W))
    //val gpio_intr_o = Output(UInt(32.W))
    val done = Output(Bool())
  })

  implicit val config: WishboneConfig = WishboneConfig(32, 32)
  val wb_imem_host = Module(new WishboneHost())
  val wb_imem_slave = Module(new WishboneDevice())
  val wb_dmem_host = Module(new WishboneHost())
  val wb_dmem_slave = Module(new WishboneDevice())
  val wb_gpio_slave = Module(new WishboneDevice())
  // val imem = Module(BlockRam.createNonMaskableRAM(programFile = programFile, bus=config, rows=1024))
  // val dmem = Module(BlockRam.createMaskableRAM(bus=config, rows=1024))
  val dmem = Module(new SRAM1kb(new WBRequest, new WBResponse)(None))
  val imem = Module(new SRAM1kb(new WBRequest, new WBResponse)(None))
  val gpio = Module(new Gpio(new WBRequest(), new WBResponse()))
  val wbErr = Module(new WishboneErr())
  // val core = Module(new Core())
  val core = Module(new Core(new WBRequest(), new WBResponse())) // NucleusRV
  // core.io.stall := false.B


  val addressMap = new AddressMap
  addressMap.addDevice(Peripherals.DCCM, "h40000000".U(32.W), "h00000FFF".U(32.W), wb_dmem_slave)
  addressMap.addDevice(Peripherals.GPIO, "h40001000".U(32.W), "h00000FFF".U(32.W), wb_gpio_slave)
  val devices = addressMap.getDevices

  val switch = Module(new Switch1toN(new WishboneMaster(), new WishboneSlave(), devices.size))

  // PUART
  val puart = Module(new PUart)
  puart.io.rxd := io.rx_i
  puart.io.CLK_PER_BIT := 4.U//io.CLK_PER_BIT

  core.io.stall :=  false.B
  puart.io.isStalled   :=  false.B

  val idle :: read_uart :: write_iccm :: prog_finish :: Nil = Enum(4)
  val state = RegInit(idle)
  val reset_reg = RegInit(true.B)
  reset_reg := reset.asBool()
  val rx_data_reg                   =       RegInit(0.U(32.W))
  val rx_addr_reg                   =       RegInit(0.U(32.W))
  // val  state_check = RegInit(0.B)
  io.done := puart.io.done
  when(~puart.io.done){
    wb_imem_host.io.reqIn.bits.addrRequest := 0.U
    wb_imem_host.io.reqIn.bits.dataRequest := 0.U
    wb_imem_host.io.reqIn.bits.activeByteLane := 0xffff.U
    wb_imem_host.io.reqIn.bits.isWrite := 0.B
    wb_imem_host.io.reqIn.valid := 0.B
    wb_imem_host.io.rspOut.ready := true.B
    when(state === idle){
      // checking to see if the reset button was pressed previously and now it falls back to 0 for starting the read uart condition
        when(reset_reg === true.B && reset.asBool() === false.B) {
            state :=  read_uart
        }.otherwise {
            state := idle
        }

        // setting all we_i to be false, since nothing to be written
        // instr_we.foreach(w => w := false.B)
        wb_imem_host.io.reqIn.valid := false.B
        //instr_we                       :=       false.B  // active high
        // instr_addr                     :=       iccm_wb_device.io.addr_o
        // instr_wdata                    :=       DontCare
        core.io.stall :=  true.B
        puart.io.isStalled   :=  false.B
    }
    .elsewhen(state === read_uart){
        // state_check := 0.B
        // when valid 32 bits available the next state would be writing into the ICCM.
        when(puart.io.valid) {
            state                    :=       write_iccm

        }.elsewhen(puart.io.done) {
            // if getting done signal it means the read_uart state got a special ending instruction which means the
            // program is finish and no need to write to the iccm so the next state would be prog_finish

            state                  :=       prog_finish

        }.otherwise {
            // if not getting valid or done it means the 32 bits have not yet been read by the UART.
            // so the next state would still be read_uart

            state                  :=       read_uart
        }

        // setting all we_i to be false, since nothing to be written
        // instr_we.foreach(w => w := false.B)
        wb_imem_host.io.reqIn.valid := false.B
        core.io.stall           :=       true.B
        puart.io.isStalled              :=       true.B

        // store data and addr in registers if uart_ctrl.valid is high to save it since going to next state i.e write_iccm
        // will take one more cycle which may make the received data and addr invalid since by then another data and addr
        // could be written inside it.

        rx_data_reg                    :=       Mux(puart.io.valid, puart.io.rx_data_o, 0.U)
        //    rx_addr_reg                    :=       Mux(puart.io.valid, puart.io.addr_o << 2, 0.U)    // left shifting address by 2 since uart ctrl sends address in 0,1,2... format but we need it in word aligned so 1 translated to 4, 2 translates to 8 (dffram requirement)
        rx_addr_reg                    :=       Mux(puart.io.valid, puart.io.addr_o, 0.U)
    }
    .elsewhen(state === write_iccm){
      // when writing to the iccm state checking if the uart received the ending instruction. If it does then
      // the next state would be prog_finish and if it doesn't then we move to the read_uart state again to
      // read the next instruction
        when(puart.io.done) {

            state                   :=       prog_finish

        }.otherwise {

            state                   :=       read_uart

        }
        // setting all we_i to be true, since instruction (32 bit) needs to be written
        // instr_we.foreach(w => w := true.B)
        wb_imem_host.io.reqIn.valid := true.B
        wb_imem_host.io.reqIn.bits.addrRequest := rx_addr_reg
        wb_imem_host.io.reqIn.bits.dataRequest := rx_data_reg
        wb_imem_host.io.reqIn.bits.activeByteLane := 0xffff.U
        wb_imem_host.io.reqIn.bits.isWrite := 1.B
        // keep stalling the core
        core.io.stall           :=       true.B
        puart.io.isStalled         :=       true.B
    }
    .elsewhen(state === prog_finish){
        // setting all we_i to be false, since nothing to be written
        // instr_we.foreach(w => w := false.B)
        wb_imem_host.io.reqIn.valid := false.B
        //instr_we                       :=       true.B   // active low
        // instr_wdata                    :=       DontCare
        // instr_addr                     :=       iccm_wb_device.io.addr_o
        core.io.stall       :=       false.B
        puart.io.isStalled         :=       false.B
        state                      :=       idle
        // state_check := 1.B
    }

    core.io.imemRsp.bits.dataResponse := 0.U
    core.io.imemRsp.bits.error := 0.B
    // core.io.imemRsp.bits.ackWrite := 0.B
    core.io.imemRsp.valid := 0.B
    core.io.imemReq.ready := true.B

  }
  .otherwise{
        wb_imem_host.io.reqIn <> core.io.imemReq
        core.io.imemRsp <> wb_imem_host.io.rspOut
  }

  // WB <-> Core (fetch)
  // wb_imem_host.io.reqIn <> core.io.imemReq
  // core.io.imemRsp <> wb_imem_host.io.rspOut
  wb_imem_slave.io.reqOut <> imem.io.req
  wb_imem_slave.io.rspIn <> imem.io.rsp

  // WB <-> WB (fetch)
  wb_imem_host.io.wbMasterTransmitter <> wb_imem_slave.io.wbMasterReceiver
  wb_imem_slave.io.wbSlaveTransmitter <> wb_imem_host.io.wbSlaveReceiver

  // WB <-> Core (memory)
  wb_dmem_host.io.reqIn <> core.io.dmemReq
  core.io.dmemRsp <> wb_dmem_host.io.rspOut
  wb_dmem_slave.io.reqOut <> dmem.io.req
  wb_dmem_slave.io.rspIn <> dmem.io.rsp


  // Switch connection
  switch.io.hostIn <> wb_dmem_host.io.wbMasterTransmitter
  switch.io.hostOut <> wb_dmem_host.io.wbSlaveReceiver
  for (i <- 0 until devices.size) {
    switch.io.devIn(devices(i)._2.litValue().toInt) <> devices(i)._1.asInstanceOf[WishboneDevice].io.wbSlaveTransmitter
    switch.io.devOut(devices(i)._2.litValue().toInt) <> devices(i)._1.asInstanceOf[WishboneDevice].io.wbMasterReceiver
  }
  switch.io.devIn(devices.size) <> wbErr.io.wbSlaveTransmitter
  switch.io.devOut(devices.size) <> wbErr.io.wbMasterReceiver
  switch.io.devSel := BusDecoder.decode(wb_dmem_host.io.wbMasterTransmitter.bits.adr, addressMap)


  wb_gpio_slave.io.reqOut <> gpio.io.req
  wb_gpio_slave.io.rspIn <> gpio.io.rsp

  io.gpio_o := gpio.io.cio_gpio_o(3,0)
  io.gpio_en_o := gpio.io.cio_gpio_en_o(3,0)
  gpio.io.cio_gpio_i := io.gpio_i

  // core.io.stall_core_i := false.B
  // core.io.irq_external_i := false.B


}

object PicofoxyDriver extends App {
  (new ChiselStage).emitVerilog(new Picofoxy(None))
}