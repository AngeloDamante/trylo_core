clean:
	rm -rf build
	rm -rf log
	rm -rf install

build_gpio:
	colcon build --packages-select trylo_gpio

build_vision:
	colcon build --packages-select trylo_vision

build_launch:
	colcon build --packages-select trylo_launch

build_control:
	colcon build --packages-select trylo_control