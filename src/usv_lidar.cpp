#include "usv_lidar.h"

void LIDARPointCloud::startTVRecord() {
    tvRecord.clear();
    isTVRecord = true;
    return;
}

void LIDARPointCloud::endTVRecord() {
    isTVRecord = false;
    return;
}

void LIDARPointCloud::calcBestRecord() {
    vector<double> tvRecordAngle;
    // vector<double> tvRecordLen;
    // vector<double> tvRecordWid;
    // vector<double> tvRecordLWRatio;
    // vector<double> dLWRatio;

    tvRecordAngle.reserve(4000);
    // tvRecordLen.reserve(4000);
    // tvRecordWid.reserve(4000);
    // tvRecordLWRatio.reserve(4000);
    // dLWRatio.reserve(4000);

    for (int i = 0; i < static_cast<int>(tvRecord.size()); i++) {
        tvRecordAngle.push_back(tvRecord[i][1]);
        // tvRecordLen.push_back(tvRecord[i][2]);
        // tvRecordWid.push_back(tvRecord[i][3]);
        // tvRecordLWRatio.push_back(tvRecord[i][4]);
    }

    // double mRecordLen = median(tvRecordLen);
    // double mRecordWid = median(tvRecordWid);
    // double estLWRatio = mRecordLen / mRecordWid;
    // for (int i = 0; i < static_cast<int>(tvRecord.size()); i++) {
    //     dLWRatio.push_back(fabs(tvRecordLWRatio[i] - estLWRatio));
    // }

    tvRecordBestAngle = 1 * median(tvRecordAngle);
            
    return;
}

void LIDARPointCloud::cloudCut(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double xLB, double xUB, double yLB, double yUB, double zLB, double zUB) {
    // 判断点云是否为空
    if (cloud->size() == 0) {
        return;
    }
    
    // 定义裁剪对象
    pcl::PassThrough<pcl::PointXYZ> passX;
    pcl::PassThrough<pcl::PointXYZ> passY;
    pcl::PassThrough<pcl::PointXYZ> passZ;

    // 剪裁点云
    // x轴
    passX.setInputCloud(cloud);
    passX.setFilterFieldName("x");
    passX.setFilterLimits(xLB, xUB);  // 裁剪保留区域
    passX.filter(*cloud);

    // y轴
    passY.setInputCloud(cloud);
    passY.setFilterFieldName("y");
    passY.setFilterLimits(yLB, yUB);
    passY.filter(*cloud);

    // z轴
    passZ.setInputCloud(cloud);
    passZ.setFilterFieldName("z");
    passZ.setFilterLimits(zLB, zUB);
    passZ.filter(*cloud);

    return;
}

void LIDARPointCloud::cloudFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double divFilter) {

    // 判断点云是否为空
    if (cloud->size() == 0) {
        return;
    }
    
    // 创建离群点滤波器对象
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;

    // 定义 VoxelGrid 滤波器变量
    pcl::VoxelGrid<pcl::PointXYZ> voxGrid;

    // 去除离群点
    sor.setInputCloud(cloud);

    // 设置在进行统计时考虑查询点邻近点数
    sor.setMeanK(100);

    // 设置判断是否为离群点的阈值
    sor.setStddevMulThresh(1.0);

    // 执行滤波处理保存内点到inputCloud
    sor.filter(*cloud);

    // 叶素滤波
    voxGrid.setInputCloud(cloud);
    voxGrid.setLeafSize(divFilter, divFilter, divFilter);
    voxGrid.filter(*cloud);

    return;
}

void LIDARPointCloud::cloudTransform(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double lidarDivX, double lidarDivY, double lidarDivZ, double lidarRotateAngle) {
    // 判断点云是否为空
    if (cloud->size() == 0) {
        return;
    }

    // 坐标转换
    for (int i = 0; i < static_cast<int>(cloud->size()); i++) {
        cloud->points[i].x = cloud->points[i].x + lidarDivX;
        cloud->points[i].y = cloud->points[i].y + lidarDivY;
        cloud->points[i].z = 0.0 + 0.0 * lidarDivZ;
        frameTF2D(cloud->points[i].x, cloud->points[i].y, lidarRotateAngle, cloud->points[i].x, cloud->points[i].y);
    }

    return;
}

void LIDARPointCloud::clusterDirection(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double objVector[3], double& len, double& wid, double& lwRatio, double& cenX, double& cenY) {
    // 判断点云是否为空
    if (cloud->size() == 0) {
        return;
    }
    
    pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
    feature_extractor.setInputCloud(cloud);
    feature_extractor.compute();

    std::vector<float> moment_of_inertia;
    std::vector<float> eccentricity;
    pcl::PointXYZ min_point_OBB;
    pcl::PointXYZ max_point_OBB;
    pcl::PointXYZ position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;
    float major_value, middle_value, minor_value;
    Eigen::Vector3f major_vector, middle_vector, minor_vector;
    Eigen::Vector3f mass_center;

    feature_extractor.getMomentOfInertia(moment_of_inertia);                                      // 计算出的惯性矩
    feature_extractor.getEccentricity(eccentricity);                                              // 计算出的偏心率
    feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);  // OBB对应的相关参数
    feature_extractor.getEigenValues(major_value, middle_value, minor_value);                     // 三个特征值
    feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);                 // 三个特征向量
    feature_extractor.getMassCenter(mass_center);                                                 // 计算质心

    // Debug: 绘制OBB包围盒
    // pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    // viewer->setBackgroundColor(0, 0, 0);
	// //viewer->addCoordinateSystem(1.0);

	// pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> RandomColor(cloud);//设置随机颜色
	// viewer->addPointCloud<pcl::PointXYZ>(cloud, RandomColor, "points");
	// viewer->setPointCl && calcNorm2(usvPosePtr->x, usvPosePtr->y) <= 100oudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "points");

	// Eigen::Vector3f position(position_OBB.x, position_OBB.y, position_OBB.z);
	// Eigen::Quaternionf quat(rotational_matrix_OBB);
	// viewer->addCube(position, quat, max_point_OBB.x - min_point_OBB.x, max_point_OBB.y - min_point_OBB.y, max_point_OBB.z - min_point_OBB.z, "OBB");
	// viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "OBB");

	// pcl::PointXYZ center(mass_center(0), mass_center(1), mass_center(2));
	// pcl::PointXYZ x_axis(major_vector(0) + mass_center(0), major_vector(1) + mass_center(1), major_vector(2) + mass_center(2));
	// pcl::PointXYZ y_axis(middle_vector(0) + mass_center(0), middle_vector(1) + mass_center(1), middle_vector(2) + mass_center(2));
	// pcl::PointXYZ z_axis(minor_vector(0) + mass_center(0), minor_vector(1) + mass_center(1), minor_vector(2) + mass_center(2));
	// viewer->addLine(center, x_axis, 1.0f, 0.0f, 0.0f, "major eigen vector");//主成分
	// viewer->addLine(center, y_axis, 0.0f, 1.0f, 0.0f, "middle eigen vector");
	// viewer->addLine(center, z_axis, 0.0f, 0.0f, 1.0f, "minor eigen vector");

    // while (!viewer->wasStopped()) {
    //     viewer->spinOnce(100);
    // }
    
    for (int i = 0; i < 3; i++) {
        objVector[i] = major_vector[i];
    }
    
    len = max_point_OBB.x - min_point_OBB.x;
    wid = max_point_OBB.y - min_point_OBB.y;
    lwRatio = len/wid;
    cenX = mass_center[0];
    cenY = mass_center[1];

    // printf("Center: [%.2f, %.2f, %.2f] | Direction: [%.2f, %.2f, %.2f]\n", mass_center[0], mass_center[1], mass_center[2], major_vector[0], major_vector[1], major_vector[2]);

    return;
}

void LIDARPointCloud::objExtract(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    // 判断点云是否为空
    if (cloud->size() == 0) {
        return;
    }
    
    // 点云欧式聚类分割，基于 KdTree 对象作为搜索方法
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdTree(new pcl::search::KdTree<pcl::PointXYZ>);
    kdTree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ecExtraction;
    ecExtraction.setClusterTolerance(10);      // 点之间相隔大于10m的不被认为是一个聚类
    ecExtraction.setMinClusterSize(40);       // 一个聚类里所允许的最少点数
    ecExtraction.setMaxClusterSize(50000);    // 一个聚类里所允许的最多点数
    ecExtraction.setSearchMethod(kdTree);     // 设置搜索方法为 KdTreee
    ecExtraction.setInputCloud(cloud);     // 设置被搜索的点云

    // 聚类抽取结果保存在一个数组中，数组中每个元素代表抽取的一个组件点云的下标
    ecExtraction.extract(clusterIndices);

    // Debug: 输出有多少个聚类
    // printf("共找到 %ld 个船只/目标\n", clusterIndices.size());

    // 遍历抽取到的所有聚类，计算相关量
    double objVector[3] = {0.0, 0.0, 0.0};
    double len = 0.0;
    double wid = 0.0;
    double lwRatio = 0.0;
    double cenX = 0.0;
    double cenY = 0.0;
    obsXYR.clear();  // 首先清除障碍物信息组的信息，准备填充新的障碍物信息
    for (std::vector<pcl::PointIndices>::const_iterator it = clusterIndices.begin(); it != clusterIndices.end(); ++it) {
        // 创建临时保存点云簇的点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudCluster(new pcl::PointCloud<pcl::PointXYZ>);

        // 通过下标，逐个填充
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++) {
            cloudCluster->points.push_back(cloud->points[*pit]);
        }

        // 计算 OBB 方向包围盒
        clusterDirection(cloudCluster, objVector, len, wid, lwRatio, cenX, cenY);

        double cenENUX = 0;
        double cenENUY = 0;
        frameTF2D(cenX, cenY, -usvPosePtr->yaw, cenENUX, cenENUY);

        // 判断是否为目标船
        if (calcNorm2(usvPosePtr->x + cenENUX, usvPosePtr->y + cenENUY) <= tvPredictR) {
            // 是目标船，则将其写入目标船的信息组中
            tvRectL = len;
            tvRectW = wid;
            tvRectLWRatio = lwRatio;
            tvX = cenX;
            tvY = cenY;
            tvAngle = atan(objVector[1] / objVector[0]) + usvPosePtr->yaw;
            USV2TargetVesselHeading = atan2(usvPosePtr->y - tvY, usvPosePtr->x - tvX);
            isLidarFindTV = true;
        } else if (calcNorm2(cenX, cenY) <= 100.0) {
            // 不是目标船，则将 XY 坐标写入障碍物的信息组中
            obsXYR.push_back({cenX, cenY, 1.2 * calcNorm2(0.5 * tvRectL, 0.5 * tvRectW)});
        }

        // Debug: 输出目标船只信息
        // printf("船只%ld：中心位于[%.2f, %.2f]m，长%.2fm，宽%.2fm，长宽比%.2f，朝向%.2fdeg\n", it - clusterIndices.begin() + 1, tvX, tvY, tvRectL, tvRectW, tvRectLWRatio, rtvAngle);
    }
}

// LIDAR callback function
void LIDARPointCloud::lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    string lidarFrameID = msg->header.frame_id;

    // 判断点云是哪个传感器的
    if (lidarFrameID == "usv/sensor_0/sensor_link/lidar" && msg->data.size() > 1) {
        pcl::fromROSMsg(*msg, *cloud0Ptr);
        cloudCut(cloud0Ptr, -150.0, 150.0, -150.0, 150.0, -10.0, 40.0);
        cloudFilter(cloud0Ptr, 0.05);
        cloudTransform(cloud0Ptr, SENSOR0_DIVX, SENSOR0_DIVY, SENSOR0_DIVZ, deg2rad(SENSOR0_ROTANG));
        isCloud0Ready = true;
        timeStampSecsLidar0 = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    } else if (lidarFrameID == "usv/sensor_2/sensor_link/lidar" && msg->data.size() > 1) {
        pcl::fromROSMsg(*msg, *cloud2Ptr);
        cloudCut(cloud2Ptr, 0.0, 150.0, -150.0, 150.0, -10.0, 40.0);
        cloudFilter(cloud2Ptr, 0.05);
        cloudTransform(cloud2Ptr, SENSOR2_DIVX, SENSOR2_DIVY, SENSOR2_DIVZ, deg2rad(SENSOR2_ROTANG));
        isCloud2Ready = true;
        timeStampSecsLidar2 = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    } else if (lidarFrameID == "usv/sensor_3/sensor_link/lidar" && msg->data.size() > 1) {
        pcl::fromROSMsg(*msg, *cloud3Ptr);
        cloudCut(cloud3Ptr, 0.0, 150.0, -150.0, 150.0, -10.0, 40.0);
        cloudFilter(cloud3Ptr, 0.05);
        cloudTransform(cloud3Ptr, SENSOR3_DIVX, SENSOR3_DIVY, SENSOR3_DIVZ, deg2rad(SENSOR3_ROTANG));
        isCloud3Ready = true;
        timeStampSecsLidar3 = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    }

    // 如果三个传感器的点云都获得，那么开始合并点云，并求解点云聚类的最小方向外接盒OBB
    if (isCloud0Ready && isCloud2Ready && isCloud3Ready) {
        // 合并点云
        *cloudPtr = *cloud0Ptr + *cloud2Ptr + *cloud3Ptr;

        // Debug: 总点云可视化
        // pcl::visualization::CloudViewer view("cloud_viewer");
        // view.showCloud(cloudPtr);
        // while (!view.wasStopped()) {
        // }

        timeStampSecs = (timeStampSecsLidar0 + timeStampSecsLidar2 + timeStampSecsLidar3) / 3.0;
        isCloud0Ready = false;
        isCloud2Ready = false;
        isCloud3Ready = false;

        // 提取目标船或障碍物的信息
        objExtract(cloudPtr);

        // 如果找到目标船，则更新 usvPose 与激光雷达有关的状态量
        if (isLidarFindTV) {
            usvPosePtr->updateLidarXY(tvX, tvY, timeStampSecs);
        }

        // 如果记录功能打开，目标船已找到并且接受到的目标船信息满足要求，则写入
        if (isLidarFindTV && isTVRecord && tvRectL >= 10 && tvRectW >= 5 && tvRectLWRatio >= 2.5) {
            tvRecord.push_back({timeStampSecs, tvAngle, tvRectL, tvRectW, tvRectLWRatio});
        }

    } else {

    }

    return;
}