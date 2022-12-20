# robot_tester

first you clone the ros package inside "catkin_ws/src"

```bash
git clone https://github.com/riotu-lab/robot_tester.git
```
Then you need to compile

```bash
catkin_make
source ~/.bashrc
```

finally you lunch the package using: 

```bash
roslaunch robot_tester test.launch host_value:=167.172.94.216 port_value:=5672 
```