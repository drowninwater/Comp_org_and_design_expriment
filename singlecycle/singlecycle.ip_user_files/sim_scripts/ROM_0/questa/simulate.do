onbreak {quit -f}
onerror {quit -f}

vsim -lib xil_defaultlib ROM_0_opt

do {wave.do}

view wave
view structure
view signals

do {ROM_0.udo}

run -all

quit -force
