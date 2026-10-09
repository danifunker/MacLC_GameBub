# Vivado Lab: program over JTAG (optional) and take ILA captures to CSV.
#   vivado_lab -mode batch -source ila_capture.tcl -tclargs <bit|-> <ltx> <outdir>
# Pass "-" as <bit> to attach to the running design without reprogramming.
set bit [lindex $argv 0]
set ltx [lindex $argv 1]
set out [lindex $argv 2]
file mkdir $out

open_hw_manager
connect_hw_server -allow_non_jtag
open_hw_target [lindex [get_hw_targets] 0]
set dev [lindex [get_hw_devices xc7a100t*] 0]
current_hw_device $dev
set_property PROBES.FILE $ltx $dev
set_property FULL_PROBES.FILE $ltx $dev
if {$bit ne "-"} {
    set_property PROGRAM.FILE $bit $dev
    program_hw_devices $dev
    puts "PROGRAMMED $bit"
}
refresh_hw_device $dev
set ila [lindex [get_hw_ilas -of_objects $dev] 0]
if {$ila eq ""} { puts "NO_ILA_FOUND"; exit 1 }
foreach p [get_hw_probes -of_objects $ila] { puts "HWPROBE $p width=[get_property WIDTH $p]" }
set depth [get_property CONTROL.DATA_DEPTH $ila]
puts "ILA depth $depth"

proc clear_triggers {ila} {
    foreach p [get_hw_probes -of_objects $ila] {
        # Data-only probes have no trigger comparator.
        catch {set_property TRIGGER_COMPARE_VALUE "eq[get_property WIDTH $p]'h[string repeat X [expr {([get_property WIDTH $p] + 3) / 4}]]" $p}
    }
}

proc capture {ila name out trigger_probe value} {
    clear_triggers $ila
    if {$trigger_probe eq ""} {
        set_property CONTROL.TRIGGER_POSITION 0 $ila
        run_hw_ila -trigger_now $ila
    } else {
        set p [get_hw_probes -of_objects $ila -filter "NAME =~ *$trigger_probe*"]
        set_property TRIGGER_COMPARE_VALUE $value $p
        set_property CONTROL.TRIGGER_POSITION [expr {[get_property CONTROL.DATA_DEPTH $ila] / 4}] $ila
        run_hw_ila $ila
    }
    if {[catch {wait_on_hw_ila -timeout 1 $ila} err]} {
        puts "CAPTURE_TIMEOUT $name ($err)"
        catch {reset_hw_ila $ila}
        return
    }
    write_hw_ila_data -csv_file -force $out/$name.csv [upload_hw_ila_data $ila]
    puts "CAPTURED $name"
}

capture $ila now      $out "" ""
capture $ila vsync_fall $out vsync "eq1'bF"
capture $ila spi_cs   $out spi_cs "eq1'bF"
puts "CAPTURE_DONE"
