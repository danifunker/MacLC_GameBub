import chisel3._
import net.gamebub.framework.Core
import net.gamebub.framework.interface._

/**
 * Macintosh LC for Game Bub.
 *
 * This class only declares which framework interfaces the core uses and how
 * they are configured. All of the logic lives in Verilog/SystemVerilog under
 * rtl/: the framework binds this IO bundle to the `maclc_gamebub` module
 * (rtl/gamebub/maclc_gamebub.sv) and prints that module's port template
 * during the build.
 */
class MacLC extends Module with Core {
  // One MMCM generates every clock (rtl/gamebub/gb_clocks.sv):
  // 50 MHz x 13 = 650 MHz VCO.
  val vcoHz = 650_000_000
  // 32.5 MHz is not negotiable: rtl/maclc/v8_clocks.sv hard-codes it, and the
  // VIA, Egret and SCC timings are derived from it.
  val systemHz = vcoHz / 20
  // The LCD clock must keep the framework's ILI9806E line at <= 612 clocks
  // (floor(framePeriod * displayHz / 808)). Above that, its back-porch clamp
  // runs with the wrong sign (amount = hBackPorchMax - hBackPorch < 0), DE is
  // set but never cleared, and the panel shows nothing (seen on hardware with
  // 32.5 MHz: 668 clocks, DE falls at x = 721 > 667). 29.55 MHz gives 607.
  val displayHz = vcoHz / 22
  // The MCU SPI sampling clock must be at least 160 MHz.
  val spiHz = vcoHz / 4

  // The LC 12" RGB monitor timing the V8 produces: 640 x 407 total at the
  // 15.6672 MHz dot clock = 60.15 Hz.
  val framePeriod = 640.0 * 407.0 / 15_667_200.0

  val io = IO(new Bundle {
    val clocks = new ClocksV0(
      clockSystemHz = systemHz,
      clockDisplayHz = displayHz,
      clockSpiHz = spiHz,
    )
    // The framework double-buffers this image in on-chip block RAM, which
    // makes it the largest RAM user on the chip: 2 x 512 x 384 x 9 bits.
    // 640x480 or deeper colour does not fit (see docs/PORT_PLAN.md).
    val video = new VideoV0(
      videoWidth = 512,
      videoHeight = 384,
      colorDepthR = 3,
      colorDepthG = 3,
      colorDepthB = 3,
      framePeriod = framePeriod,
    )
    val audio = new AudioV0()
    val host = new HostV0()
    val input = new InputV0()
    // Mac RAM, ROM and every disk image (memory map: rtl/gamebub/gb_host.sv).
    val sdram = new SdramV0(chips = 1)
    // 512 KiB async SRAM = the LC's 512 KB VRAM option.
    val sram = new SramV0(addressWidth = 18, dataWidth = 16)
    // Mac modem port (serial/MIDI) and the debug beacon, 3.3 V.
    val pmod = new PmodV0()
  })

  // The host gives the core no wall-clock time, so the Mac's clock starts
  // at the moment the bitstream was built.
  val buildUnixTime = (System.currentTimeMillis() / 1000).toInt

  bindExtModule("maclc_gamebub", io, Map(
    "BUILD_UNIX_TIME" -> buildUnixTime,
  ))
}
