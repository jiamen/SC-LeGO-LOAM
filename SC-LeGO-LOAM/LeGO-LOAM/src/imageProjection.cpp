//
// Created by zlc on 2021/5/13.
//
// Description:
// 总体流程：
//      订阅点云数据回调处理->点云转换到pcl预处理->截取一帧激光数据->投影映射到图像->地面移除->点云分割->发布点云数据->重置参数;


#include "utility.h"

class ImageProjection
{
private:

    ros::NodeHandle nh;                     // 句柄， 启动关闭节点； 指定命名空间

    ros::Subscriber subLaserCloud;          // 订阅雷达点云

    ros::Publisher pubFullCloud;            // 发布全部点云
    ros::Publisher pubFullInfoCloud;        // 发布全部点云信息,与上面的FullCloud相比多了 强度（Intensity）和 range ??????

    ros::Publisher pubGroundCloud;          // 发布地面点云
    ros::Publisher pubSegmentedCloud;       // 发布分割点云
    ros::Publisher pubSegmentedCloudPure;
    ros::Publisher pubSegmentedCloudInfo;
    ros::Publisher pubOutlierCloud;

    // 将输入的ROS传感器点云数据结构    转换    下面的一些PCL点云数据结构
    pcl::PointCloud<PointType>::Ptr  laserCloudIn;      // 接受到的来自激光Msg的原始点云数据
    pcl::PointCloud<PointXYZIR>::Ptr laserCloudInRing;  // 用 laserCloudInRing 存储含有具有通道R的原始点云数据

    // 深度图点云：以一维形式存储与深度图像素一一对应的点云数据
    pcl::PointCloud<PointType>::Ptr fullCloud;          // projected velodyne raw cloud, but saved in the form of 1-D matrix
    // I值 = intensity = (float) rowIdn + (float) columnIdn / 10000.0;
    // 带距离值的深度图点云:与深度图点云存储一致的数据，但是其属性intensity记录的是距离值,   range = x²+y²+z² ,最小的点与雷达的深度
    pcl::PointCloud<PointType>::Ptr fullInfoCloud;      // same as fullCloud, but with intensity - range  每个点的
    // I值 = intensity = range

    // 注：所有点分为被分割点、未被分割点、地面点、无效点。
    pcl::PointCloud<PointType>::Ptr groundCloud;        // 地面点点云
    pcl::PointCloud<PointType>::Ptr segmentedCloud;     // segMsg 点云数据:包含被分割点和经过降采样的地面点，intensity=
    pcl::PointCloud<PointType>::Ptr segmentedCloudPure; // 可视化点订阅，存储被分割点点云，且每个点的i值为分割标志值
    pcl::PointCloud<PointType>::Ptr outlierCloud;       // 经过降采样的未分割点

    // 每次fullCloud和fullInfoCloud的点云初始化就会填入下面这种数据，这构造函数中进行初始化
    PointType nanPoint;     // fill in fullCLoud at each iteration

    cv::Mat rangeMat;       // range matrix for range image     点云转化为深度图，每个点的三维坐标转换为单通道深度值
    cv::Mat labelMat;       // label matrix for segmentation marking
    cv::Mat groundMat;      // ground matrix for ground cloud marking
    int labelCount;

    float startOrientation;         // 起始朝向
    float endOrientation;           // 结束朝向

    // 自创的点云ros数据结构  segMsg点云信息(存储分割结果并用于发送)
    cloud_msgs::cloud_info segMsg;  // info of segmented cloud  有存储点云是否为地面的标志
    std_msgs::Header cloudHeader;   // 雷达点云头

    // 每个点的邻域索引
    std::vector<std::pair<int8_t, int8_t>> neighborIterator;

    uint16_t* allPushedIndX;        // array for tracking points of a segmented object. 当前点所在的物体分割类的所有索引，隶属于同一个被分割的物体
    uint16_t* allPushedIndY;

    uint16_t* queueIndX;            // array for breadth-first search process of segmentation, for speed
    uint16_t* queueIndY;


public:
    ImageProjection() : nh("~")
    {
        subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2> (pointCloudTopic, 1, &ImageProjection::cloudHandler, this);

        pubFullCloud  = nh.advertise<sensor_msgs::PointCloud2>("/full_cloud_projected", 1);
        pubFullInfoCloud = nh.advertise<sensor_msgs::PointCloud2>("/full_cloud_info", 1);

        pubGroundCloud = nh.advertise<sensor_msgs::PointCloud2>("/ground_cloud", 1);
        pubSegmentedCloud = nh.advertise<sensor_msgs::PointCloud2>("/segmented_cloud", 1);
        pubSegmentedCloudPure = nh.advertise<sensor_msgs::PointCloud2>("/segmented_cloud_pure", 1);
        pubSegmentedCloudInfo = nh.advertise<cloud_msgs::cloud_info> ("/segmented_cloud_info", 1);
        pubOutlierCloud = nh.advertise<sensor_msgs::PointCloud2> ("/outlier_cloud", 1);

        // 初始化中每次都给fullInfoCloud和fullCloud填入的点
        nanPoint.x = std::numeric_limits<float>::quiet_NaN();
        nanPoint.y = std::numeric_limits<float>::quiet_NaN();
        nanPoint.z = std::numeric_limits<float>::quiet_NaN();
        nanPoint.intensity = -1;

        allocateMemory();
        resetParameters();
    }

    ~ImageProjection()  {  }

    void allocateMemory()
    {
        laserCloudIn.reset(new pcl::PointCloud<PointType>());
        laserCloudInRing.reset(new pcl::PointCloud<PointXYZIR>());

        fullCloud.reset(new pcl::PointCloud<PointType>());
        fullInfoCloud.reset(new pcl::PointCloud<PointType>());

        groundCloud.reset(new pcl::PointCloud<PointType>());
        segmentedCloud.reset(new pcl::PointCloud<PointType>());
        segmentedCloudPure.reset(new pcl::PointCloud<PointType>());
        outlierCloud.reset(new pcl::PointCloud<PointType>());

        fullCloud->points.resize(N_SCAN*Horizon_SCAN);
        fullInfoCloud->points.resize(N_SCAN*Horizon_SCAN);

        segMsg.startRingIndex.assign(N_SCAN, 0);
        segMsg.endRingIndex.assign(N_SCAN, 0);

        segMsg.segmentedCloudGroundFlag.assign(N_SCAN*Horizon_SCAN, false);
        segMsg.segmentedCloudColInd.assign(N_SCAN*Horizon_SCAN, 0);
        segMsg.segmentedCloudRange.assign( N_SCAN*Horizon_SCAN, 0);

        // 将每个点的邻域关系存入 邻域索引向量（数组）
        std::pair<int8_t, int8_t> neighbor;
        neighbor.first = -1; neighbor.second =  0;  neighborIterator.push_back(neighbor);
        neighbor.first =  0; neighbor.second =  1;  neighborIterator.push_back(neighbor);
        neighbor.first =  0; neighbor.second = -1;  neighborIterator.push_back(neighbor);
        neighbor.first =  1; neighbor.second =  0;  neighborIterator.push_back(neighbor);

        allPushedIndX = new uint16_t[N_SCAN*Horizon_SCAN];
        allPushedIndY = new uint16_t [N_SCAN*Horizon_SCAN];

        // 所有点的（x,y）索引坐标，注意数组大小
        queueIndX = new uint16_t[N_SCAN*Horizon_SCAN];
        queueIndY = new uint16_t[N_SCAN*Horizon_SCAN];
    }

    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
    {
        // 1. Convert ros message to pcl point cloud    复制点云数据
        copyPointCloud(laserCloudMsg);  // 将ROS中的sensor_msgs::PointCloud2ConstPtr类型转换到pcl点云库指针。

        // 2. Start and end angle of a scan 找到开始和结束的角度
        findStartEndAngle();            // ①一圈数据的角度差，使用atan2计算; ②注意计算结果的范围合理性

        // 3. Range image projection    点云投影
        projectPointCloud();

        // 4. Mark ground points        地面检测，移出地面点
        groundRemoval();

        // 5. Point cloud segmentation  点云分割
        cloudSegmentation();

        // 6. Publish all clouds        发布处理后的点云数据
        publishCloud();

        // 7. Reset parameters for next iteration   重置参数，清空此次的点云变量
        resetParameters();
    }

    // 回调函数第1步：把ros点云消息数据结构 转换为 PCL点云数据结构
    void copyPointCloud(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
    {
        cloudHeader = laserCloudMsg->header;        // 保存雷达点云头
        cloudHeader.stamp = ros::Time::now();       // Ouster lidar users may need to uncomment this line
        pcl::fromROSMsg(*laserCloudMsg, *laserCloudIn);     //

        // Remove Nan points
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*laserCloudIn, *laserCloudIn, indices);

        // have "ring" channel in the cloud   有线圈数
        if (useCloudRing == true)
        {
            pcl::fromROSMsg(*laserCloudMsg, *laserCloudInRing);
            if (laserCloudInRing->is_dense == false)
            {
                ROS_ERROR("Point cloud is not in dense format, please remove NaN points first!");
                ros::shutdown();
            }
        }
    }

    // 回调函数第2步：找到开始和结束的角度， 将起始点与最末点进行角度的转换
    void findStartEndAngle()
    {
        // start and end orientation of this cloud
        // 计算角度时以x轴负轴为基准
        segMsg.startOrientation = -atan2(laserCloudIn->points[0].y, laserCloudIn->points[0].x);
        // 因此最末角度为2π减去计算值
        segMsg.endOrientation   = -atan2(laserCloudIn->points[laserCloudIn->points.size() - 1].y,
                                       laserCloudIn->points[laserCloudIn->points.size() - 1].x) + 2 * M_PI;

        if (segMsg.endOrientation-segMsg.startOrientation > 3*M_PI)
        {
            segMsg.endOrientation -= 2*M_PI;        // 减去一圈
        }
        else if (segMsg.endOrientation-segMsg.startOrientation < M_PI)
        {
            segMsg.endOrientation += 2*M_PI;        // 加上一圈
        }
        segMsg.orientationDiff = segMsg.endOrientation - segMsg.startOrientation;
    }

    // 回调函数第3步： 点云投影 , 逐一计算点云深度，并具有深度的点云保存至fullInfoCloud中
    void projectPointCloud()
    {
        // range image projection
        float verticalAngle, horizonAngle, range;
        size_t rowIdn, columnIdn, index, cloudSize;
        PointType thisPoint;

        cloudSize = laserCloudIn->points.size();

        // 输入激光雷达点云以一维向量存储，所以直接遍历，处理所有点云
        for (size_t i=0; i<cloudSize; i ++)
        {
            thisPoint.x = laserCloudIn->points[i].x;
            thisPoint.y = laserCloudIn->points[i].y;
            thisPoint.z = laserCloudIn->points[i].z;

            // 计算竖直方向上的点的角度以及在整个雷达点云中的哪一条水平线上
            // find the row and column index in the image for this point.
            if (useCloudRing == true)
            {
                rowIdn = laserCloudInRing->points[i].ring;
            }
            else
            {
                verticalAngle = atan2(thisPoint.z, sqrt(thisPoint.x*thisPoint.x + thisPoint.y*thisPoint.y)) * 180 / M_PI;
                rowIdn = (verticalAngle + ang_bottom) / ang_res_y;          // 找到是垂直哪一行，得到点的行号
            }
            // 出现异常角度则无视
            if (rowIdn<0 || rowIdn>=N_SCAN)
                continue;

            // 注意查看 VELODYNE 雷达数据， 这里在一圈中角度结合上面在哪一行可以确定点的位置
            // △△△△△△   下面这几句很不好理解：总体意思是根据角度得到一条线上的点索引    □□□□□□□□□
            horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;

            columnIdn = -round((horizonAngle-90.0)/ang_res_x) + Horizon_SCAN/2;
            if (columnIdn >= Horizon_SCAN)
                columnIdn -= Horizon_SCAN;

            if (columnIdn<0 || columnIdn>=Horizon_SCAN)
                continue;

            // 当前点与雷达的深度
            range = sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y + thisPoint.z * thisPoint.z);
            if (range < sensorMinimumRange)
                continue;
            // 在深度图rangeMat矩阵中保存该点的深度，保存单通道像素值
            rangeMat.at<float>(rowIdn, columnIdn) = range;

            thisPoint.intensity = (float) rowIdn + (float) columnIdn / 10000.0;     // 当前点的深度

            index = columnIdn + rowIdn * Horizon_SCAN;          // 当前点的单通道索引值
            fullCloud->points[index] = thisPoint;               // fullCloud也是单通道索引
            fullInfoCloud->points[index] = thisPoint;           // fullInfoCloud
            fullInfoCloud->points[index].intensity = range;     // the corresponding range of a point is saved as "intensity"
        }
    }

    // 回调函数第4步：区分地面点云与非地面点云
    void groundRemoval()
    {
        size_t lowerInd, upperInd;
        float diffX, diffY, diffZ, angle;

        // groundMat
        // -1, no valid info to check if ground or not
        // 0, initial value, after validation, means not ground
        // 1, ground after validation

        for (size_t j=0; j<Horizon_SCAN; j ++)              // 一圈中的哪个点
        {
            for (size_t i=0; i<groundScanInd; i ++)         // 在哪一条线上
            {
                lowerInd = j + (i)*Horizon_SCAN;
                upperInd = j + (i+1)*Horizon_SCAN;

                if (fullCloud->points[lowerInd].intensity == -1 ||
                    fullCloud->points[upperInd].intensity == -1)
                {
                    groundMat.at<int8_t>(i, j) = -1;
                    continue;           // 无法验证点，直接跳过
                }

                diffX = fullCloud->points[upperInd].x - fullCloud->points[lowerInd].x;
                diffY = fullCloud->points[upperInd].y - fullCloud->points[lowerInd].y;
                diffZ = fullCloud->points[upperInd].z - fullCloud->points[lowerInd].z;

                angle = atan2(diffZ, sqrt(diffX*diffX + diffY*diffY)) * 180 / M_PI;

                if (abs(angle-sensorMountAngle) <= 10)
                {
                    groundMat.at<int8_t>(i, j) = 1;
                    groundMat.at<int8_t>(i+1, j) = 1;
                }
            }
        }

        // extract ground cloud (groundMat == 1)  从所有点云中提取地面点云
        // mark entry that doesn't need to label (ground and invalid point) for segmentation
        // note that ground remove is from 0~N_SCAN-1, need rangeMat for mark label matrix for the 16th scan
        for (size_t i=0; i<N_SCAN; i ++)                // 64条线
        {
            for (size_t j=0; j<Horizon_SCAN; j ++)      // 每条线上1024个点
            {
                // 如果是地面点云 或者 是无效点，labelMat矩阵中标记为-1
                if (groundMat.at<int8_t>(i, j) == 1 || rangeMat.at<float>(i,j) == FLT_MAX)
                {
                    labelMat.at<int>(i, j) = -1;
                }
            }
        }
        // 如果发布的地面点云有订阅者的话，用PCL点云数据结构专门存储地面点云
        if (pubGroundCloud.getNumSubscribers() != 0)
        {
            for (size_t i=0; i<=groundScanInd; i ++)
            {
                for (size_t j=0; j<Horizon_SCAN; j ++)
                {
                    if (groundMat.at<int8_t>(i, j) == 1)
                    {
                        groundCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                    }
                }
            }
        }
    }

    // 回调函数第5步：点云分割，包含每个点的label标签 标记
    void cloudSegmentation()
    {
        // segmentation process
        for (size_t i=0; i<N_SCAN; i ++)                // 在哪一条线上
        {
            for (size_t j=0; j<Horizon_SCAN; j ++)      // 一条线上的哪个点
            {
                if (labelMat.at<int>(i, j) == 0)
                    labelComponents(i, j);
            }
        }

        int sizeOfSegCloud = 0;         // 已分割点云的数量
        // extract segmented cloud for lidar odometry
        // 为雷达里程计提取分割过的点云
        for (size_t i=0; i<N_SCAN; i ++)
        {
            segMsg.startRingIndex[i] = sizeOfSegCloud-1 + 5;    // 每个环起始点的坐标索引，还扣掉了5个点

            for (size_t j=0; j<Horizon_SCAN; j ++)
            {
                // 如果标签矩阵不为0，或者已经是地面点，就存入待发布的
                if (labelMat.at<int>(i,j) > 0 || groundMat.at<int8_t>(i,j)==1)
                {
                    // outliers that will not be used for optimization (always continue)
                    // 进一步判断 标签矩阵中值 是否合理
                    if (labelMat.at<int>(i,j) == 999999)
                    {
                        // 离群点选取在高于地面15根线之上，并且列索引是5的倍数
                        if (i>groundScanInd && j%5 == 0)
                        {
                            outlierCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                            continue;
                        }
                        else
                        {
                            continue;
                        }
                    }
                    // majority of ground points are skipped    地面点云降采样
                    if (groundMat.at<int8_t>(i,j) == 1)
                    {
                        if (j%5!=0 && j>5 && j<Horizon_SCAN-5)
                            continue;
                    }
                    // mark ground points so they will not be considered as edge features later
                    // 保存是否为地面，如果是地面的话，后面就不会再被考虑成边特征
                    segMsg.segmentedCloudGroundFlag[sizeOfSegCloud] = (groundMat.at<int8_t>(i,j) == 1);
                    // mark the points' column index for marking occlusion later
                    segMsg.segmentedCloudColInd[sizeOfSegCloud] = j;
                    // save range info  保存距离信息
                    segMsg.segmentedCloudRange[sizeOfSegCloud]  = rangeMat.at<float>(i,j);

                    // save seg cloud   保存点云原xyzI信息
                    segmentedCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);

                    // size of seg cloud  已分割点云数量 + 1
                    ++ sizeOfSegCloud;
                }
            }

            segMsg.endRingIndex[i] = sizeOfSegCloud-1 - 5;      // 每个环结束点的坐标索引，还扣掉了5个点
        }

        // extract segmented cloud for visualization，
        if (pubSegmentedCloudPure.getNumSubscribers() != 0)
        {
            for (size_t i=0; i<N_SCAN; i ++)
            {
                for (size_t j=0; j<Horizon_SCAN; j ++)
                {
                    if (labelMat.at<int>(i,j) >0 && labelMat.at<int>(i,j) != 999999)
                    {
                        segmentedCloudPure->push_back(fullCloud->points[j+i*Horizon_SCAN]);
                        segmentedCloudPure->points.back().intensity = labelMat.at<int>(i, j);
                    }
                }
            }
        }
    }

    // 回调函数第6步：发布处理后的点云数据
    void publishCloud()
    {
        // 1. Publish Seg Cloud Info  使用自创的点云数据结构segMsg
        segMsg.header = cloudHeader;
        pubSegmentedCloudInfo.publish(segMsg);
        // 2. Publish clouds
        sensor_msgs::PointCloud2 laserCloudTemp;

        // 发布15条地线上面的离群点
        pcl::toROSMsg(*outlierCloud, laserCloudTemp);
        laserCloudTemp.header.stamp = cloudHeader.stamp;        // 时间戳
        laserCloudTemp.header.frame_id = "base_link";
        pubOutlierCloud.publish(laserCloudTemp);

        // segmented cloud with ground，包含地面的分割点云
        pcl::toROSMsg(*segmentedCloud, laserCloudTemp);
        laserCloudTemp.header.stamp = cloudHeader.stamp;
        laserCloudTemp.header.frame_id = "base_link";
        pubSegmentedCloud.publish(laserCloudTemp);

        // projected full cloud   全部点云
        if (pubFullCloud.getNumSubscribers() != 0)
        {
            pcl::toROSMsg(*fullCloud, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "base_link";
            pubFullCloud.publish(laserCloudTemp);
        }
        // original dense ground cloud  稠密地面点云
        if (pubGroundCloud.getNumSubscribers() != 0)
        {
            pcl::toROSMsg(*groundCloud, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "base_link";
            pubGroundCloud.publish(laserCloudTemp);
        }
        // original dense ground cloud
        if (pubSegmentedCloudPure.getNumSubscribers() != 0)
        {
            pcl::toROSMsg(*segmentedCloudPure, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "base_link";
            pubSegmentedCloudPure.publish(laserCloudTemp);
        }
        // projected full cloud info
        if (pubFullInfoCloud.getNumSubscribers() != 0)
        {
            pcl::toROSMsg(*fullInfoCloud, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "base_link";
            pubFullInfoCloud.publish(laserCloudTemp);
        }
    }

    // 回调函数第7步：重置参数，清空此次的点云变量
    void resetParameters()
    {
        laserCloudIn->clear();              // 最初接收的激光雷达点云清空
        groundCloud->clear();
        segmentedCloud->clear();
        segmentedCloudPure->clear();
        outlierCloud->clear();              // 离散点清空

        rangeMat  = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));
        groundMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_8S, cv::Scalar::all(0));
        labelMat  = cv::Mat(N_SCAN, Horizon_SCAN, CV_32S, cv::Scalar::all(0));
        labelCount = 1;                     // 分割后有几个标签

        std::fill(fullCloud->points.begin(), fullCloud->points.end(), nanPoint);
        std::fill(fullInfoCloud->points.begin(), fullInfoCloud->points.end(), nanPoint);
    }


    void labelComponents(int row, int col)
    {
        // use std::queue std::vector std::deque will slow the program down greatly
        float d1, d2, alpha, angle;
        int fromIndX, fromIndY, thisIndX, thisIndY;
        bool lineCountFlag[N_SCAN] = {false};

        queueIndX[0] = row;             // 行，在哪条线上
        queueIndY[0] = col;             // 列，在一条线的哪个点上

        int queueSize = 1;              // 队列中点的数量
        int queueStartInd = 0;          // 队列起始索引
        int queueEndInd = 1;            // 队列结束索引

        allPushedIndX[0] = row;
        allPushedIndY[0] = col;
        int allPushedIndSize = 1;

        while (queueSize > 0)
        {
            // Pop point，起始点的（X，Y）坐标
            fromIndX = queueIndX[queueStartInd];
            fromIndY = queueIndY[queueStartInd];
            -- queueSize;
            ++ queueStartInd;
            // Mark popped point  标记当前点的标签号，把这个点当作中心点，遍历上下所有四个邻域点
            labelMat.at<int>(fromIndX, fromIndY) = labelCount;
            // Loop through all the neighboring grids of popped grid
            for (auto iter=neighborIterator.begin(); iter!=neighborIterator.end(); iter ++)
            {
                // new index
                thisIndX = fromIndX + (*iter).first;
                thisIndY = fromIndY + (*iter).second;

                // index should be within the boundary
                if (thisIndX<0 || thisIndX>=N_SCAN)
                    continue;

                // at range image margin (left or right side)  注意是一个圈，值在[0, 1024]之间循环
                if (thisIndY < 0)
                    thisIndY = Horizon_SCAN - 1;
                if (thisIndY >= Horizon_SCAN)
                    thisIndY = 0;

                // prevent infinite loop (caused by put already examined point back)  如果本点已经有标记了，则跳过，防止无限循环
                if (labelMat.at<int>(thisIndX, thisIndY) != 0)
                    continue;

                // 比较邻域点和中心点与激光雷达的距离大小
                d1 = std::max(rangeMat.at<float>(fromIndX, thisIndY),
                              rangeMat.at<float>(thisIndX, thisIndY));
                d2 = std::min(rangeMat.at<float>(fromIndX, fromIndY),
                              rangeMat.at<float>(thisIndX, thisIndY));

                if ((*iter).first == 0)
                    alpha = segmentAlphaX;
                else
                    alpha = segmentAlphaY;

                angle = atan2(d2*sin(alpha), (d1-d2* cos(alpha)));

                // 如果 β 大于一定的阈值，说明两者之间没有突变，所以可以认为是同一个聚类
                if (angle > segmentTheta)       // segmentTheta = π/3
                {
                    queueIndX[queueEndInd] = thisIndX;
                    queueIndY[queueEndInd] = thisIndY;
                    ++ queueSize;               // 队列长度加1， while继续循环
                    ++ queueEndInd;             // 结束队列索引 后移

                    labelMat.at<int>(thisIndX, thisIndY) = labelCount;
                    lineCountFlag[thisIndX] = true;

                    allPushedIndX[allPushedIndSize] = thisIndX;         // 当前点所在的物体分割类的所有索引都存到allPushIndX、Y中
                    allPushedIndY[allPushedIndSize] = thisIndY;
                    ++ allPushedIndSize;
                }
            }
        }

        // check if this segment is valid   检查这次分割是否有效
        bool feasibleSegment = false;
        if (allPushedIndSize >= 30)
            feasibleSegment = true;
        else if (allPushedIndSize >= segmentValidPointNum)
        {
            int lineCount = 0;
            for (size_t i=0; i<N_SCAN; i ++)
                if (lineCountFlag[i] == true)
                    lineCount ++;
            if (lineCount >= segmentValidLineNum)   // segmentValidLineNum=3
                feasibleSegment = true;
        }

        // segment is valid, mark these points      分割是有效的，标记上面的点，并且把标签labelCount++
        if (feasibleSegment == true)
        {
            ++ labelCount;
        }
        else
        {
            for (size_t i=0; i < allPushedIndSize; i ++)
            {
                labelMat.at<int>(allPushedIndX[i], allPushedIndY[i]) = 999999;
            }
        }
    }


};


int main(int argc, char* *argv)
{
    ros::init(argc, argv, "lego_loam");

    ImageProjection IP;
    // ros::NodeHandle private_nh("~");         // 句柄，  nh命名空间为/node_namespace/lego_loam/

    ROS_INFO("\033[1;32m---->\033[0m Image Projection Started.");

    ros::spin();

    return 0;
}


