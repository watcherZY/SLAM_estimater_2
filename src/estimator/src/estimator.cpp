//#include "backend.h"
#include "../include/estimator.h" 

Estimator::Estimator(): f_manager{Rs}
{
    ROS_INFO("init begins");
    clearState();
}

void Estimator::setParameter()
{
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        tic[i] = TIC[i];
        ric[i] = RIC[i];
    }
    f_manager.setRic(ric);
    ProjectionFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    ProjectionTdFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    td = TD;
}

void Estimator::clearState()
{
    if(SHOW_DETAIL_INFO)
        ROS_INFO("--------------clearState-----------");

    for (int i = 0; i < WINDOW_SIZE + 1; i++)
    {
        Rs[i].setIdentity();
        Ps[i].setZero();
        Vs[i].setZero();
        Bas[i].setZero();
        Bgs[i].setZero();
        dt_buf[i].clear();
        linear_acceleration_buf[i].clear();
        angular_velocity_buf[i].clear();

        if (pre_integrations[i] != nullptr)
            delete pre_integrations[i];
        pre_integrations[i] = nullptr;
    }

    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        tic[i] = Vector3d::Zero();
        ric[i] = Matrix3d::Identity();
    }

    for (auto &it : all_image_frame)
    {
        if (it.second.pre_integration != nullptr)
        {
            delete it.second.pre_integration;
            it.second.pre_integration = nullptr;
        }
    }

    solver_flag = INITIAL;
    first_imu = false,
    //sum_of_back = 0;
    //sum_of_front = 0;
    frame_count = 0;
//    solver_flag = INITIAL;
    initial_timestamp = 0;
    all_image_frame.clear();
    td = TD;


    if (tmp_pre_integration != nullptr)
        delete tmp_pre_integration;
//    if (last_marginalization_info != nullptr)
//        delete last_marginalization_info;

    tmp_pre_integration = nullptr;
//    last_marginalization_info = nullptr;
//    last_marginalization_parameter_blocks.clear();

    f_manager.clearState();

    failure_occur = 0;
    relocalization_info = 0;

    drift_correct_r = Matrix3d::Identity();
    drift_correct_t = Vector3d::Zero();
}

void Estimator::processIMU(double dt, const Vector3d &linear_acceleration, const Vector3d &angular_velocity)
{   /* dt为本次imu和上次imu的采样间隔
        acc本次的加速度测量值
        ang本次的角速度测量值 */
    
    if (!first_imu)
    {
        first_imu = true;
        acc_0 = linear_acceleration;
        gyr_0 = angular_velocity;
    }
    if(SHOW_DETAIL_INFO){
        ROS_INFO("frame_count is %d",frame_count);
    }
    if (!pre_integrations[frame_count])
    {
        pre_integrations[frame_count] = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};
         ROS_INFO("=====================pre_integrations  %d  construction==================",frame_count);
        
    }
    if (frame_count != 0)
    {
        pre_integrations[frame_count]->push_back(dt, linear_acceleration, angular_velocity);//预积分
        /* if(SHOW_DETAIL_INFO){
            cout<<"pre_integrations delta_p is "<<pre_integrations[frame_count]->delta_p<<endl;
        } */
        if(solver_flag != NON_LINEAR)
            tmp_pre_integration->push_back(dt, linear_acceleration, angular_velocity);

        dt_buf[frame_count].push_back(dt);
        linear_acceleration_buf[frame_count].push_back(linear_acceleration);
        angular_velocity_buf[frame_count].push_back(angular_velocity);

        int j = frame_count;         
        Vector3d un_acc_0 = Rs[j] * (acc_0 - Bas[j]) - g;
        Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - Bgs[j];
        Rs[j] *= Utility::deltaQ(un_gyr * dt).toRotationMatrix();
        Vector3d un_acc_1 = Rs[j] * (linear_acceleration - Bas[j]) - g;
        Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
        Ps[j] += dt * Vs[j] + 0.5 * dt * dt * un_acc;
        Vs[j] += dt * un_acc;
    }
    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
}

void Estimator::processImage(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, const std_msgs::Header &header)
{
    // 1. decide marg_old or marg new and add features to f_manager
    ROS_DEBUG("new image coming ------------------------------------------");
    ROS_DEBUG("Adding feature points %lu", image.size());
    if (f_manager.addFeatureCheckParallax(frame_count, image, td))
        marginalization_flag = MARGIN_OLD;
    else
        marginalization_flag = MARGIN_SECOND_NEW;

    // 2. build all_image_frame for initialization
    ROS_DEBUG("this frame is--------------------%s", marginalization_flag ? "reject" : "accept");
    ROS_DEBUG("%s", marginalization_flag ? "Non-keyframe" : "Keyframe");
    ROS_DEBUG("Solving %d", frame_count);
    ROS_DEBUG("number of feature: %d", f_manager.getFeatureCount());
    Headers[frame_count] = header;

    if (solver_flag != NON_LINEAR) 
    {
        ImageFrame imageframe(image, header.stamp.toSec());
        imageframe.pre_integration = tmp_pre_integration;
        all_image_frame.insert(make_pair(header.stamp.toSec(), imageframe));
        tmp_pre_integration = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};
    }

    // 3. calibrate the rotation and translation from camera to IMU
    //calibrationExRotation();

    //4. initialize
    if (solver_flag == INITIAL){
        initial(header);
        
    } 
    //5. optimization
    else{
        backend.backend(this);
    }
        
}

void Estimator::calibrationExRotation()
{
    if(ESTIMATE_EXTRINSIC == 2)
    {
        ROS_INFO("calibrating extrinsic param, rotation movement is needed");
        if (frame_count != 0)
        {
            vector<pair<Vector3d, Vector3d>> corres = f_manager.getCorresponding(frame_count - 1, frame_count);
            Matrix3d calib_ric;
            if (initial_ex_rotation.CalibrationExRotation(corres, pre_integrations[frame_count]->delta_q, calib_ric))
            {
                ROS_WARN("initial extrinsic rotation calib success");
                ROS_WARN_STREAM("initial extrinsic rotation: " << endl << calib_ric);
                ric[0] = calib_ric;
                RIC[0] = calib_ric;
                ESTIMATE_EXTRINSIC = 1;
            }
        }
    }
}

void Estimator::initial(const std_msgs::Header &header)
{
    if (frame_count == WINDOW_SIZE)
    {
        bool result = false;
        //要保证这一帧和上一帧有一定的时间用来运动
        if((header.stamp.toSec() - initial_timestamp) < 0.2){
            ROS_INFO("too close of frame %f and %f !!!",header.stamp.toSec(),initial_timestamp);
        }
            
        if( ESTIMATE_EXTRINSIC != 2 && (header.stamp.toSec() - initial_timestamp) >= 0.2)
        {
            ROS_INFO("begin initialStructure... ");
            result = intializer.initialStructure(this);
            initial_timestamp = header.stamp.toSec();
        }
        /* 上一步的初始化过程并不是一步到位的，在前期的一系列运行之后，才得到 */
        if(!result){
            ROS_INFO("initialStructure failed!!!");
        }
        if(result)
        {
            solver_flag = NON_LINEAR;
            ROS_INFO("start backend.solveOdometry && backend.slideWindow");
            backend.solveOdometry(this);
            backend.slideWindow(this);
            f_manager.removeFailures();
            ROS_INFO("Initialization finish!");
            last_R = Rs[WINDOW_SIZE];
            last_P = Ps[WINDOW_SIZE];
            last_R0 = Rs[0];
            last_P0 = Ps[0];               
        }
        else{
            ROS_INFO("backend.slideWindow after initialStructure failed");
            backend.slideWindow(this);
        }
    }
    else
        frame_count++;
}

void Estimator::setReloFrame(double _frame_stamp, int _frame_index, vector<Vector3d> &_match_points, Vector3d _relo_t, Matrix3d _relo_r)
{
    relo_frame_stamp = _frame_stamp;
    relo_frame_index = _frame_index;
    match_points.clear();
    match_points = _match_points;
    prev_relo_t = _relo_t;
    prev_relo_r = _relo_r;
    for(int i = 0; i < WINDOW_SIZE; i++)
    {
        if(relo_frame_stamp == Headers[i].stamp.toSec())
        {
            relo_frame_local_index = i;
            relocalization_info = 1;
            for (int j = 0; j < SIZE_POSE; j++)
                relo_Pose[j] = backend.para_Pose[i][j];
        }
    }
}

