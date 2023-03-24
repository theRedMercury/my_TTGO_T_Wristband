#  addr2line -f -e .pio/build/esp32dev/firmware.elf 

check:
	pio check --skip-packages

clean:
	find include/ src/ lib/ -regex '.*\.\(cpp\|hpp\|h\|c\)' -exec clang-format -style=file -i {} \;

