FW_PATH=../firmware/
VID=0x16c0
PID=0x072f

scan: 
	./fx2-programmer any any dump_busses

program:
	./fx2-programmer $(VID) $(PID) set 0xE600 1
	./fx2-programmer $(VID) $(PID) program $(FW_PATH)gn3s_firmware.ihx
	./fx2-programmer $(VID) $(PID) set 0xE600 0

read:
	./fx2-programmer $(VID) $(PID) read_vendor

