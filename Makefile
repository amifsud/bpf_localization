.ONESHELL:
make:
	cwd=$$PWD
	ws=`echo $$ROS_PACKAGE_PATH | awk -F: '{print $$1}'`
	echo "ws="$$ws
	cd $$ws/..
	catkin_make
	echo "We where in this workspace : "$$PWD
