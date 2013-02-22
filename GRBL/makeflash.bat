rem Runs MAKE, flashes MINI-AT board on serial port [Param1]
make all
avrdude -C "avrdude.conf" -pm328p -cstk500v1 -V -P COM%1 -b57600 -D -U flash:w:"GRBL.HEX":i 