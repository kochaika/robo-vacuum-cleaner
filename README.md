# robo-vacuum-cleaner
Типа "робот-пылесос" на основе turtlebot3

## Зависимости
Требуется поставить: turtlebot3_gazebo, ros-kinetic-turtlebot3-simulations
```
sudo apt-get install ros-kinetic-turtlebot3-gazebo ros-kinetic-turtlebot3-simulations ros-kinetic-turtlebot3-description

```
## Как запустить?
Сначала установить переменную среды:
```
export TURTLEBOT3_MODEL=burger
```
После для запуска мира и модели робота:
```
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```
Для запуска логики работы
```
roslaunch robocleaner robocleaner.launch
```
