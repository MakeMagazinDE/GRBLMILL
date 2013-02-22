rem Flashes MINI-AT board on serial port [Param1]
avrdude -C "avrdude.conf" -pm328p -cstk500v1 -P COM%1 -b57600 -D -U flash:w:"GRBL.HEX":i 