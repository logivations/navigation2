# Navigation

```bash
sudo docker pull logivations/ml_all:ros_nav2
sudo docker run -ti logivations/ml_all:ros_nav2 bash
```
##### In docker container
```bash
source /code/nav2_depend_ws/install/setup.bash
cd /data/workspace/navigation2_ws/src
rm navigation_2 -rf
git clone https://github.com/andriimaistruk/navigation2
cd ..
colcon build --packages-select nav2_core nav2_planner nav2_costmap_2d nav2_util nav2_lifecycle_manager smac_planner nav2_navfn_planner nav2_common nav2_msgs
source install/setup.bash
ros2 run nav2_planner planner_server
```
#### Now you have planner_server running  
#### How to send goal to it  
##### open a new terminal session
###### assumption is that you have deep_cv setup ;=)
```bash
sudo docker exec -ti deep_cv bash
cd /data/workspace/deep_cv
git checkout task_wmo_55759_investigate_smac_hybrid_astar_planner
```
##### For deep_cv/appconfig/tracking/agv_simulation_config.xml the id must be set 1 (in planner_server that is hardcoded), this a temporal measure ;=)
##### Lauch run_tracking and run_simulation using pycharm gui )

#### Make planner_server transition to activated state
##### open a new terminal session
```bash
sudo docker run -ti logivations/ml_all:ros_nav2 bash
```
##### In docker container
```bash
ros2 lifecycle set /nav2_planner configure
ros2 lifecycle set /nav2_planner activate
```
##### Use "Send to" button in W2MO 3D to send goal to AGV. The visualized path is computed by planner_server 