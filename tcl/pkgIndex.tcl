package ifneeded "Linuxcnc" 1.0 \
 [list source [file join $dir linuxcnc.tcl]]
package ifneeded "Hal" 1.0 \
 [list load [file join $dir hal[info sharedlibextension]]]
