.PHONY: test make

.ONESHELL:
make:
	cwd=$$PWD
	ws=`echo $$ROS_PACKAGE_PATH | awk -F: '{print $$1}'`
	cd $$ws/..
	catkin_make

.ONESHELL:
test:
	make
	cwd=$$PWD
	ws=`echo $$ROS_PACKAGE_PATH | awk -F: '{print $$1}'`
	cd $$ws/..
	catkin_make run_tests_bpf_localization_gtest
