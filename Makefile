# ==============================================================================
# 编译校验用 Makefile（arm-none-eabi-gcc）
#
# 说明：
#   这份 Makefile 只把每个 .c 编译成 .o，用来验证代码在 GCC 下能过编译，
#   不做链接、不产出 bin/hex。正式固件仍然用 Keil MDK 打开 balance_car.uvprojx
#   编译和下载，两者互不影响。
#
# 用法：
#   make          # 编译全部源文件
#   make clean    # 清理 build/
# ==============================================================================

TARGET  := balance_car
BUILD   := build

CC      := arm-none-eabi-gcc
MCU     := -mcpu=cortex-m3 -mthumb
DEFS    := -D USE_STDPERIPH_DRIVER -D STM32F10X_MD
INCS    := -Iuser -Imy_lib -Itest -Istd_periph_driver/inc -Ici
CFLAGS  := $(MCU) $(DEFS) $(INCS) -std=gnu11 -Os -Wall \
           -ffunction-sections -fdata-sections

SRCS    := $(wildcard user/*.c) $(wildcard my_lib/*.c) $(wildcard test/*.c) \
           $(wildcard std_periph_driver/src/*.c)
OBJS    := $(patsubst %.c,$(BUILD)/%.o,$(SRCS))

all: $(OBJS)
	@echo ""
	@echo "== 编译校验通过：$(words $(OBJS)) 个源文件 =="

$(BUILD)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -rf $(BUILD)

.PHONY: all clean
