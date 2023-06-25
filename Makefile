clean:
	rm -rf build
	rm -rf log
	rm -rf install

build_gpio:
	colcon build --packages-select trylo_gpio

build_motor:
	colcon build --packages-select trylo_motor