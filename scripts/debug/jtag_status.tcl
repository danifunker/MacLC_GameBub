# Read-only: identify the JTAG chain and read the FPGA's configuration status.
open_hw_manager
connect_hw_server -allow_non_jtag
set targets [get_hw_targets -quiet]
puts "TARGETS: $targets"
foreach t $targets {
    if {[catch {open_hw_target $t} err]} { puts "OPEN_FAILED $t: $err"; continue }
    foreach d [get_hw_devices -quiet] {
        puts "DEVICE: $d  PART=[get_property -quiet PART $d]"
        if {[catch {refresh_hw_device -update_hw_probes false $d} err]} { puts "REFRESH_FAILED: $err" }
        foreach p [lsort [list_property $d]] {
            if {[string match "REGISTER.CONFIG_STATUS*" $p] || [string match "REGISTER.USERCODE*" $p] || [string match "REGISTER.USR_ACCESS*" $p] || [string match "REGISTER.IR*" $p] || [string match "REGISTER.BOOT_STATUS*" $p] || [string match "*TEMPERATURE*" $p] || $p eq "IDCODE_HEX" || $p eq "PROGRAM.IS_SUPPORTED" || $p eq "DID"} {
                puts "PROP $p = [get_property -quiet $p $d]"
            }
        }
    }
    close_hw_target $t
}
puts "DONE"
