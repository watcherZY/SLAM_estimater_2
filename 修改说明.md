版本2：
实现了bias_acc，bias_gyro的估计功能，并且把轨迹进行了输出。
后端优化和原版的没有区别
vins_result_no_loop的输出格式：

```
<< estimator.Ps[WINDOW_SIZE].x() << ","
<< estimator.Ps[WINDOW_SIZE].y() << ","
<< estimator.Ps[WINDOW_SIZE].z() << ","
<< tmp_Q.w() << ","
<< tmp_Q.x() << ","
<< tmp_Q.y() << ","
<< tmp_Q.z() << ","
<< estimator.Vs[WINDOW_SIZE].x() << ","
<< estimator.Vs[WINDOW_SIZE].y() << ","
<< estimator.Vs[WINDOW_SIZE].z() << ","

```
使用方法：
source devel/setup.bash
roslaunch vi_slam run.launch



