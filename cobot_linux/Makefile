.PHONY: all build clean run

BUILD_DIR := build

all: build

build:
	@mkdir -p $(BUILD_DIR)
	cd $(BUILD_DIR) && cmake .. && make -j$(nproc)

clean:
	rm -rf $(BUILD_DIR) install log
