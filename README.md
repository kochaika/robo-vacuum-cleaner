# robo-vacuum-cleaner
Типа "робот-пылесос" на основе turtlebot3

## Зависимости
Требуется поставить: turtlebot3_gazebo, ros-kinetic-turtlebot3-simulations
```
sudo apt-get install ros-kinetic-turtlebot3-gazebo ros-kinetic-turtlebot3-simulations ros-kinetic-turtlebot3-description

```
## Как запустить?

Урезать дальность лидара
```
roscd turtlebot3_description
sudo nano urdf/turtlebot3_burger.gazebo.xacro
```
```
<range>
          <min>0.10</min>
          <max>0.4</max>
```
Указать форму робота
```
export TURTLEBOT3_MODEL=burger
```
Модель мира и робот
```
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```
Отображение карты
```
rosrun rviz rviz -d `rospack find turtlebot3_slam`/rviz/turtlebot3_slam.rviz
```
SLAM
```
roslaunch turtlebot3_slam turtlebot3_slam.launch
```
логика работы писоса
```
roslaunch robocleaner robocleaner.launch
```
Если хочется рулить писосом самостоятельно
```
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```
