#!/bin/bash

if [[ $1 = "--local" ]]
then
    dir=$2
    if [[ $dir = "" ]]
    then
	echo "error: usage[./setup.sh --local robot_dir]"
	exit 1
    else
	echo "setup local $dir"
    fi
else
    dir=$1
    if [[ $dir = "" ]]
    then
	echo "error: no robot specified"
	exit 1
    else
	echo "setup robot $dir"
    fi
fi

pkg=$(echo $dir | cut -d/ -f1)
dir=$(echo $dir | cut -d/ -f2)

if [[ -z "${dir}" ]]; then
    # compensate args
    # from: ./setup.sh robot_dir/
    # to  : ./setup.sh robot_dir
    dir=${pkg}
fi

if [ $dir != $pkg ]
then
    ln -s $(rospack find $pkg)/$dir .
fi

upper_file=$(ls ./$dir | grep "upper" | grep ".txt")
if [[ $upper_file = "" ]]
then
    echo "error: make sure robot directory exists"
    echo "error: upper file must match pattern *upper*.txt"
    exit 1
fi

upper_name=$(echo "$upper_file" | awk -F "." '{print $1}')
lower_name=$(ls ./$dir | grep ".txt" | grep -v $upper_file | awk -F "." '{print $1}')

check_install=$(rospack find aero_description)
if [[ $check_install = "" ]]
then
    echo "error: aero_description must be built before running script"
    exit 1
fi

printf "\nCreating urdf ...\n"

./scripts/create_urdf.sh $dir $upper_name $lower_name

printf "\nCreating EusLISP model ...\n"

./scripts/create_eusmodel.sh $dir

printf "\nSetting up aero_moveit_config ...\n"

./scripts/setup_moveit_config.sh $dir

if [[ $1 = "--local" ]]
then
    printf "\nsetting up srvs ...\n"

    ./scripts/install_srv.sh
else
    printf "\nsetting up srvs ...\n"

    ./scripts/install_srv.sh

    printf "\nwriting files to aero_startup/aero_hardware_interface ...\n"

    ./scripts/make_controller.sh $dir

    ## not used
    ## printf "\nwriting files to aero_startup/aero_controller_manager ...\n"
    ## ./scripts/make_joint_state_publisher.sh $dir

    printf "\nmaking Angle2Stroke.hh ... \n"

    ./scripts/make_angle_to_stroke_header.sh $dir $upper_name $lower_name

    printf "\nmaking Stroke2Angle.hh ... \n"

    ./scripts/make_stroke_to_angle_header.sh $dir

    printf "\ncreating UnusedAngle2Stroke.hh ...\n"

    ./scripts/unused_angle_to_stroke.sh $dir

    printf "\nconfigurating controllers ...\n"

    ./scripts/configure_controllers.sh $dir
fi

if [[ $dir = "arm_typeC" ]]
then
  input_file=$(rospack find aero_ros_controller)/src/AeroHandController.cc
  input_file2=$(rospack find aero_std)/include/aero_std/AeroMoveitInterface.hh
  echo "overwriting AeroHandController.cc"
  sed -i -e "36,48s:^://:" $input_file
  echo "renewing AeroMoveitInterface.hh"
  sed -i.bak -e '/USING_UPPERTYPEF/ s/1/0/' $input_file2 
  fi
else
  input_file=$(rospack find aero_ros_controller)/src/AeroHandController.cc
  input_file2=$(rospack find aero_std)/include/aero_std/AeroMoveitInterface.hh
  sed -i -e "36,48 s#//###g" $input_file
  echo "renewing AeroMoveitInterface.hh"
  sed -i.bak -e '/USING_UPPERTYPEF/ s/0/1/' $input_file2 
fi

printf "\nbuild aero_startup ...\n"
catkin b aero_ros_controller
catkin b aero_startup
build_result=$?

printf "\nfinished.\n"
exit $build_result
