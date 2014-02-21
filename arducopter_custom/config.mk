
# Select 'mega' for the 1280 APM1, 'mega2560' otherwise
BOARD = mega2560

# HAL_BOARD determines default HAL target.
HAL_BOARD ?= HAL_BOARD_APM1

# The communication port used to communicate with the APM.
PORT = /dev/ttyUSB0

# uncomment and fill in the path to Arduino if installed in an exotic location
# ARDUINO = /path/to/Arduino

# PX4Firmware tree: fill in the path to PX4Firmware repository from github.com/diydrones:
#PX4_ROOT=../PX4Firmware

# PX4NuttX tree: fill in the path to PX4NuttX repository from github.com/diydrones:
#NUTTX_SRC=../PX4NuttX/nuttx
