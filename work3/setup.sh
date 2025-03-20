#!/bin/bash

# 设置工作空间路径
WS_DIR=$(pwd)
SRC_DIR=${WS_DIR}/src
LMS_FILE="../data/URG_X_20130903_195003.lms"
NAV_FILE="../data/ld.nav"

# 创建ROS2工作空间结构
mkdir -p ${SRC_DIR}/lms_mapper/lms_mapper
mkdir -p ${SRC_DIR}/lms_mapper/resource
mkdir -p ${SRC_DIR}/lms_data/lms_data
mkdir -p ${SRC_DIR}/lms_data/resource

# 创建Python包初始化文件
touch ${SRC_DIR}/lms_data/lms_data/__init__.py
touch ${SRC_DIR}/lms_mapper/lms_mapper/__init__.py

# 创建package.xml文件
cat > ${SRC_DIR}/lms_mapper/package.xml << EOF
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>lms_mapper</name>
  <version>0.0.1</version>
  <description>LMS mapper for ROS2</description>
  <maintainer email="student@example.com">student</maintainer>
  <license>Apache License 2.0</license>

  <buildtool_depend>ament_python</buildtool_depend>
  
  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>rosbag2_py</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
EOF

cat > ${SRC_DIR}/lms_data/package.xml << EOF
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>lms_data</name>
  <version>0.0.1</version>
  <description>LMS data conversion for ROS2</description>
  <maintainer email="student@example.com">student</maintainer>
  <license>Apache License 2.0</license>

  <buildtool_depend>ament_python</buildtool_depend>
  
  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>rosbag2_py</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
EOF

# 创建setup.py文件
cat > ${SRC_DIR}/lms_mapper/setup.py << EOF
from setuptools import setup, find_packages

package_name = 'lms_mapper'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@example.com',
    description='LMS mapper for ROS2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lms_mapper_node = lms_mapper.lms_mapper_node:main',
        ],
    },
)
EOF

cat > ${SRC_DIR}/lms_data/setup.py << EOF
from setuptools import setup, find_packages

package_name = 'lms_data'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@example.com',
    description='LMS data conversion for ROS2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lms_to_rosbag = lms_data.lms_to_rosbag:main',
        ],
    },
)
EOF

# 创建资源标记文件
touch ${SRC_DIR}/lms_mapper/resource/lms_mapper
touch ${SRC_DIR}/lms_data/resource/lms_data

# 确保Python脚本有可执行权限
chmod +x ${SRC_DIR}/lms_data/lms_data/lms_to_rosbag.py
chmod +x ${SRC_DIR}/lms_mapper/lms_mapper/lms_mapper_node.py

echo "ROS2工作空间结构创建完成！"

# 清理之前的构建
rm -rf build/ install/ log/

# 重新构建
colcon build --symlink-install

# 重新设置环境
source install/setup.bash 

# 解决 "No executable found" 问题
ls -la install/lms_data/lib/lms_data/
ls -la install/lms_mapper/lib/lms_mapper/

# 在setup.sh末尾添加这些命令
mkdir -p install/lms_data/lib/lms_data/
mkdir -p install/lms_mapper/lib/lms_mapper/
ln -sf ${SRC_DIR}/lms_data/lms_data/lms_to_rosbag.py install/lms_data/lib/lms_data/lms_to_rosbag
ln -sf ${SRC_DIR}/lms_mapper/lms_mapper/lms_mapper_node.py install/lms_mapper/lib/lms_mapper/lms_mapper_node