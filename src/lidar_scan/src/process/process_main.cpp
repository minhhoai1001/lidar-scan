#include <ros/ros.h> 
#include <sensor_msgs/LaserScan.h>
#include "laser_geometry/laser_geometry.h"
#include <sensor_msgs/PointCloud.h>
#include <visualization_msgs/Marker.h>
#include <cmath>
#include "iniparser.h"
#include "polygon.h"
#include "DouglasPeucker.h"
#include "lidar_scan.h"
#include "lidar_scan/ScanCommand.h"
#include <chrono>
#include <thread>

class Object
{
public:
    unsigned int    id;
    ROBOT_TYPE    type;
    int conf;
    int x;
    int x_sub;
    int y;
    int y_sub;
    int ang;
    std::vector<float> width;
    std::vector<float> length;
    std::vector<float> angle;
    std::vector<float> x_cell;
    std::vector<float> y_cell;

    Object():
        id{0}, type{ROBOT_TYPE::unknow}, conf{0}, 
        x{0}, x_sub{0}, y{0}, y_sub{0}, ang{0},
        width{0.0}, length{0.0}, angle{0.0}, x_cell{0.0}, y_cell{0.0}
    {

    }

    float median(std::vector<float> *v)
    {
        size_t n = v->size() / 2;
        nth_element(v->begin(), v->begin()+n, v->end());
        return v->at(n);
    }

    void check_type(float w, float l)
    {
        if (l > 0.9)
            type = ROBOT_TYPE::R5_bin;
        else if ((l>0.6 && l < 0.9) || w > 0.65)
            type = ROBOT_TYPE::B1;
        else if ((l<0.6 && l>0.5) || w < 0.65)
            type = ROBOT_TYPE::R5;
        else
            type = ROBOT_TYPE::unknow;
    }

    int confidence(float w, float l)
    {        float ratio = 0.0;
        if (type == ROBOT_TYPE::B1)
        {
            ratio = std::max(w/B1_W, l/B1_L);
        }
        else if (type == ROBOT_TYPE::R5)
            ratio = std::max(w/R5_W, l/R5_L);
        else if (type == ROBOT_TYPE::R5_bin)
            ratio = std::max(w/R5_W, l/R5BIN_L);

        return static_cast<int>(ratio*100);
    }

    void calculator(st_service_res *response)
    {
        float w = median(&width);
        float l = median(&length);
        float xc = median(&x_cell);
        float yc = median(&y_cell);
        ang = median(&angle);
        check_type(w, l);
        x = int(xc);
        x_sub  = (int)((xc-x)*10000);
        y = int(yc);
        y_sub  = (int)((yc-y)*10000);
        conf = confidence(w, l);

        response->angle = ang;
        response->conf  = conf;
        response->type  = type;
        response->x     = x;
        response->x_sub = x_sub;
        response->y     = y;
        response->y_sub = y_sub;
    }

    std::string robot_name(ROBOT_TYPE type)
    {
        switch (type)
        {
        case ROBOT_TYPE::R5:
            return "R5";
        case ROBOT_TYPE::R5_bin:
            return "R5_BIN";
        case ROBOT_TYPE::B1:
            return "B1";
        default:
            return "unknow";
        }
    }

    void print()
    {
        ROS_INFO("---------- Object ID: %d ----------", id);
        ROS_INFO("Type: %s", robot_name(type).c_str());
        ROS_INFO("Angle: %d, X: %d, X_sub: %d, Y: %d, Y_sub: %d, conf: %d", ang, x, x_sub, y, y_sub, conf);
        ROS_INFO("----------------------------------");
    }

    void clean()
    {
        width.clear();
        length.clear();
        angle.clear();
        x_cell.clear();
        y_cell.clear();
    }
};

class Lidar_Scan
{
private:
    ros::Subscriber scan_sub;
    ros::Publisher cloud_pub;
    ros::Publisher marker_pub;
    ros::ServiceServer server;
    laser_geometry::LaserProjection projector;
    visualization_msgs::Marker line_list;
    visualization_msgs::Marker line_roi;
    visualization_msgs::Marker sensor;
    visualization_msgs::Marker points;

    key_t key;
    int msgid;
    long int msgtyp{0};

    char* ini_file;
    dictionary* ini;
    st_calib_sensor calib;
    st_ROI_cell roi_cell{0, 0, 0, 0};
    st_ROI_m roi_m{0.0, 0.0, 0.0, 0.0};
    st_service_req request{GRIP_TYPE::DDG, 0, 0, {0, 0, 0, 0}};
    st_service_res response{};
    st_grip_info grip_info{GRIP_TYPE::DDG, 0.505, 0.705};
    st_cluster_cfg cluster_cfg{};
    std::vector<Object> objects{};
    unsigned int cnt{0};
    bool scan_start{false};
    bool scan_done{false};
    int return_value{0};


    void calib_lidar(sensor_msgs::PointCloud* cloud)
    {
        for(unsigned int i=0; i<cloud->points.size(); i++)
        {
            float temp = cloud->points[i].x;
            cloud->points[i].x = - cloud->points[i].y + calib.x_offset;
            cloud->points[i].y = temp + calib.y_offset;
        }
    }

    int check_position_roi()
    {
        if (roi_cell.x1<=0 && roi_cell.x2>=0)
            return ROI_POSITION::opposite;
        else if(roi_cell.x2<0)
        {
            if (roi_cell.y2 >=0 && roi_cell.y1 <=0)
                return ROI_POSITION::left;
            else
                return ROI_POSITION::left_corner;
        }
        else 
        {
            if (roi_cell.y2 >=0 && roi_cell.y1 <=0)
                return ROI_POSITION::right;
            else
                return ROI_POSITION::right_corner;
        }
    }

    void AScord2Robotcord()
    {
        roi_cell.x1 = request.roi.x1 - request.robot_x;
        roi_cell.y1 = request.roi.y1 - request.robot_y;
        roi_cell.x2 = request.roi.x2 - request.robot_x;
        roi_cell.y2 = request.roi.y2 - request.robot_y;

        roi_m.x1    = roi_cell.x1 * grip_info.cell_width - grip_info.cell_width/2;
        roi_m.y1    = roi_cell.y1 * grip_info.cell_length;
        roi_m.x2    = roi_cell.x2 * grip_info.cell_width + grip_info.cell_width/2;
        roi_m.y2    = roi_cell.y2 * grip_info.cell_length;
    }

    bool check_obtacle_points(std::vector<Point>* coords, const sensor_msgs::PointCloud* cloud)
    {
        Polygon poly{*coords};
        int cnt = 0;
        for (unsigned int i=0; i<cloud->points.size(); i++)
        {
            Point p{cloud->points[i].x, cloud->points[i].y, cloud->points[i].z};
            if( poly.checkInside(&p) ) cnt +=1;
        }
        if (cnt < 50)
            return false; // no obtacle
        else
            return true;  // have obtacle
    }

    bool check_obtacle(const sensor_msgs::PointCloud* cloud)
    {
        AScord2Robotcord();
        int pos = check_position_roi();
        if (pos == ROI_POSITION::opposite)
        {
            std::vector<Point> coords = {{0.0f, 0.0f, 0.0f}, {roi_m.x1, roi_m.y1, 0.0f}, {roi_m.x2, roi_m.y1, 0.0f}};
            return check_obtacle_points(&coords, cloud);
        }
        else if (pos == ROI_POSITION::left)
        {
            std::vector<Point> coords = {{0.0f, 0.0f, 0.0f}, {roi_m.x2, roi_m.y1, 0.0f}, {roi_m.x2, roi_m.y2, 0.0f}};
            return check_obtacle_points(&coords, cloud);
        }
        else if (pos == ROI_POSITION::left_corner)
        {
            std::vector<Point> coords = {{0.0f, 0.0f, 0.0f}, {roi_m.x1, roi_m.y1, 0.0f}, {roi_m.x2, roi_m.y1, 0.0f}, {roi_m.x2, roi_m.y2, 0.0f}};
            return check_obtacle_points(&coords, cloud);
        }
        else if (pos == ROI_POSITION::right)
        {
            std::vector<Point> coords = {{0.0f, 0.0f, 0.0f}, {roi_m.x1, roi_m.y1, 0.0f}, {roi_m.x1, roi_m.y2, 0.0f}};
            return check_obtacle_points(&coords, cloud);
        }
        else if (pos == ROI_POSITION::right_corner)
        {
            std::vector<Point> coords = {{0.0f, 0.0f, 0.0f}, {roi_m.x1, roi_m.y2, 0.0f}, {roi_m.x1, roi_m.y1, 0.0f}, {roi_m.x2, roi_m.y1, 0.0f}};
            return check_obtacle_points(&coords, cloud);
        }
        return false;
    }

    void crop_ROI(sensor_msgs::PointCloud* cloud, std::vector<Point>* roi)
    {
        for (unsigned int i=0; i<cloud->points.size(); i++)
        {
            if ((cloud->points[i].x>roi_m.x1 && cloud->points[i].x<roi_m.x2) &&
                (cloud->points[i].y>roi_m.y1 && cloud->points[i].y<roi_m.y2 ))
            {
                Point p {cloud->points[i].x, cloud->points[i].y, cloud->points[i].z};
                roi->push_back(p);
            }
        }
    }

    void cluster_object(std::vector<Point>* roi, std::vector<std::vector<Point>>* cluster_obj)
    {
        std::vector<Point> cluster;
        for(unsigned int i=0; i<roi->size(); i++)
        {
            cluster.push_back(roi->at(i));
            bool is_cluster_ready = false;
            if(i==roi->size()-1)
                is_cluster_ready = true;
            else
            {
                float x_present = roi->at(i).x;
                float y_present = roi->at(i).y;
                float x_next = roi->at(i+1).x;
                float y_next = roi->at(i+1).y;

                float distance = pow(x_present-x_next, 2) + pow(y_present-y_next, 2);
                is_cluster_ready = distance > pow(cluster_cfg.distance, 2) ? true: false;
            }
            if (is_cluster_ready)
            {
                if(cluster.size() >= cluster_cfg.min_size)
                    cluster_obj->push_back(cluster);
                cluster.clear();
            }
        }
    }

    float meter2cell(float m, char t)
    {
        if (t=='x')
            return (m-grip_info.cell_width/2)/grip_info.cell_width + request.robot_x;
        else if (t=='y')
            return m/grip_info.cell_length + request.robot_y;
        else
            return 0.0;
    }

    float measure_x(float x1, float x2)
    {
        float min = std::min(x1, x2);
        if (min < 0)
            return meter2cell(std::max(x1, x2), 'x');
        else
            return meter2cell(min, 'x');
    }

    void measuring_object(std::vector<Point>& lines, Object* object)
    {
        // 1 line
        if(lines.size() == 2)
        {
            if (abs(lines[0].x-lines[1].x) >= 0.3) // horizontal line
            {
                float w = abs(lines[0].x-lines[1].x);
                float l = abs(lines[0].y-lines[1].y);
                float a = 0.0;
                if (w>0) a = atan(l/w)*180/PI;
                float x = measure_x(lines[0].x, lines[1].x);
                float y = meter2cell(std::min(lines[0].y, lines[1].y), 'y');
                object->width.push_back(w);
                object->length.push_back(0);
                object->x_cell.push_back(x);
                object->y_cell.push_back(y);
                object->angle.push_back(a);
            }
        }
        // 2 line
        else if(lines.size() == 3)
        {
            ROS_DEBUG("Two line");
            if(abs(lines[0].x-lines[1].x) >= 0.3) // horizontal line
            {
                float w = abs(lines[0].x-lines[1].x);
                float l = abs(lines[0].y-lines[1].y);
                float a = 0.0;
                if (w>0) a = atan(l/w)*180/PI;
                float x = measure_x(lines[0].x, lines[1].x);
                float y = meter2cell(std::min(lines[0].y, lines[1].y), 'y');
                object->width.push_back(w);
                object->length.push_back(abs(lines[2].y-lines[1].y));
                object->x_cell.push_back(x);
                object->y_cell.push_back(y);
                object->angle.push_back(a);
            }
            else // vertical line
            {
                float w = abs(lines[2].x-lines[1].x);
                float l = abs(lines[2].y-lines[1].y);
                float a = 0.0;
                if (w>0) a = atan(l/w)*180/PI;
                float x = measure_x(lines[2].x, lines[1].x);
                float y = meter2cell(std::min(lines[2].y, lines[1].y), 'y');
                object->width.push_back(w);
                object->length.push_back(abs(lines[0].y-lines[1].y));
                object->x_cell.push_back(x);
                object->y_cell.push_back(y);
                object->angle.push_back(a);
            }
        }
        else
        {
            ROS_DEBUG("Skip this object");
        }

    }

    void maker_cluster_point(std::vector<Point>& cluster_point)
    {
        points.header.frame_id = "cloud";
        points.header.stamp = ros::Time::now();
        points.ns = "lines";
        points.action = visualization_msgs::Marker::ADD;
        points.pose.orientation.w = 1.0;
        points.id = 3;
        points.type = visualization_msgs::Marker::POINTS;
        points.scale.x = 0.005;
        points.scale.y = 0.005;
        points.color.r = 1.0;
        points.color.a = 1.0;

        for (unsigned int i=0; i<cluster_point.size(); i++)
        {
            geometry_msgs::Point p;
            p.x = cluster_point.at(i).x;
            p.y = cluster_point.at(i).y;
            points.points.push_back(p);
        }
        marker_pub.publish(points);
    }

    void maker_roi()
    {
        line_roi.header.frame_id = "cloud";
        line_roi.header.stamp = ros::Time::now();
        line_roi.ns = "lines";
        line_roi.action = visualization_msgs::Marker::ADD;
        line_roi.pose.orientation.w = 1.0;
        line_roi.id = 0;
        line_roi.type = visualization_msgs::Marker::LINE_LIST;

        //line width
        line_roi.scale.x = 0.005;

        // Line strip is blue
        line_roi.color.g = 1.0;
        line_roi.color.a = 1.0;

        // Draw vertical line of ROI
        for(float x=roi_m.x1; x<=roi_m.x2; x+=grip_info.cell_width)
        {
            geometry_msgs::Point p1;
            p1.x = x;
            p1.y = roi_m.y1;
            line_roi.points.push_back(p1);
                        
            geometry_msgs::Point p2;
            p2.x = x;
            p2.y = roi_m.y2;
            line_roi.points.push_back(p2);
        }

        // Draw horizontal line of ROI
        for(float y=roi_m.y1; y<=roi_m.y2; y+=grip_info.cell_length)
        {
            geometry_msgs::Point p1;
            p1.x = roi_m.x1;
            p1.y = y;
            line_roi.points.push_back(p1);
                        
            geometry_msgs::Point p2;
            p2.x = roi_m.x2;
            p2.y = y;
            line_roi.points.push_back(p2);
        }

        marker_pub.publish(line_roi);

        sensor.header.frame_id = "cloud";
        sensor.header.stamp = ros::Time::now();
        sensor.ns = "lines";
        sensor.action = visualization_msgs::Marker::ADD;
        sensor.pose.orientation.w = 1.0;
        sensor.id = 1;
        sensor.type = visualization_msgs::Marker::POINTS;
        sensor.scale.x = 0.05;
        sensor.scale.y = 0.05;
        sensor.color.r = 1.0;
        sensor.color.a = 1.0;

        geometry_msgs::Point p;
        p.x = 0.0;
        p.y = 0.0;
        sensor.points.push_back(p);

        marker_pub.publish(sensor);
    }

    void maker_line(std::vector<Point> lines)
    {
        line_list.header.frame_id = "cloud";
        line_list.header.stamp = ros::Time::now();
        line_list.ns = "lines";
        line_list.action = visualization_msgs::Marker::ADD;
        line_list.pose.orientation.w = 1.0;
        line_list.id = 2;
        line_list.type = visualization_msgs::Marker::LINE_LIST;

        //line width
        line_list.scale.x = 0.01;

        // Line strip is blue
        line_list.color.b = 1.0;
        line_list.color.a = 1.0;

        for (unsigned int i=0; i<lines.size()-1; i++)
        {
            geometry_msgs::Point p;
            p.x = lines[i].x;
            p.y = lines[i].y;
            line_list.points.push_back(p);
            p.x = lines[i+1].x;
            p.y = lines[i+1].y;
            line_list.points.push_back(p);
        }
    }

    void get_config()
    {
        ROS_INFO("ini_file: %s", ini_file);
        ini = iniparser_load(ini_file);
        calib.x_offset = iniparser_getfloat(ini, "CALIB:X_OFFSET", 0.0);
        calib.y_offset = iniparser_getfloat(ini, "CALIB:Y_OFFSET", 0.35);
        cluster_cfg.epsilon     = iniparser_getfloat(ini, "CLUSTER:EPSILON", 0.035);
        cluster_cfg.distance    = iniparser_getfloat(ini, "CLUSTER:NEIGHBOR_DISTANCE", 0.044);
        cluster_cfg.min_size    = iniparser_getint(ini, "CLUSTER:MIN_CLUSTER_SIZE", 15);
    }

    void write_file()
    {
        std::string path;
        ros::param::get("output_path", path);
        std::ostringstream buf;

        auto currentTime = std::chrono::system_clock::now();
        std::time_t currentTime_t = std::chrono::system_clock::to_time_t(currentTime);
        return_value = static_cast<int>(currentTime_t);
        buf<<"/"<<currentTime_t;
        // buf<<"_grip."<<request.grip<<"_robotx_"<<request.robot_x<<"_roboty_"<<request.robot_y<<"_x1_";
        // buf<<request.roi.x1<<"_y1_"<<request.roi.y1<<"_x2_"<<request.roi.x2<<"_y2_"<<request.roi.y2;
        buf<<".txt";
        path = path + buf.str();
        ROS_INFO("PATH: %s", path.c_str());

        std::ofstream out(path, std::ios_base::app);
        out << response.type <<" ";
        out << response.angle <<" ";
        out << response.x <<" ";
        out << response.x_sub <<" ";
        out << response.y <<" ";
        out << response.y_sub <<" ";
        out << response.conf <<"\n";
        out.close();
    }

    void callback(const sensor_msgs::LaserScan::ConstPtr& scan)
    {
        if (this->scan_start)
        {
            if (scan->ranges.size() == 0)
            {
                ROS_ERROR("LIDAR ERROR: no data");
                return;
            }

            sensor_msgs::PointCloud cloud;
            projector.projectLaser(*scan, cloud);
        
            calib_lidar(&cloud);

            if (check_obtacle(&cloud))
            {
                ROS_ERROR("LIDAR ERROR: blocking scan area");
                return;
            }

            std::vector<Point> roi;
            crop_ROI(&cloud, &roi);

            std::vector<std::vector<Point>> cluster_obj;
            cluster_object(&roi, &cluster_obj);

            for (unsigned int i=0; i<cluster_obj.size(); i++)
            {
                std::vector<Point> lines = LineSegment::DouglasPeucker(cluster_obj[i], cluster_cfg.epsilon);
                maker_line(lines);

                if(cnt == 0)
                {
                    Object object;
                    object.id = i;
                    measuring_object(lines, &(object));
                    objects.push_back(object);
                }
                else
                {
                    measuring_object(lines, &(objects[i]));
                }
            }

            cnt++;

            if(cnt == 40)
            {
                ROS_INFO("Number cluster: %ld", cluster_obj.size());
                for (unsigned int i=0; i<cluster_obj.size(); i++)
                {
                    objects[i].calculator(&response);
                    objects[i].print();

                    write_file();

                    objects[i].clean();
                }
                cnt = 0;
                scan_done = true;
                scan_start = false;
                ROS_INFO("Stop scan");
            }
            
            maker_roi();

            cloud_pub.publish(cloud);
            marker_pub.publish(line_list);
        }
    }

    void get_request(lidar_scan::ScanCommand::Request &req)
    {
        request.grip    = req.grid_type;
        request.robot_x = req.x_rrt;
        request.robot_y = req.y_rrt;
        request.roi.x1  = req.x1_ROI_AS;
        request.roi.y1  = req.y1_ROI_AS;
        request.roi.x2  = req.x2_ROI_AS;
        request.roi.y2  = req.y2_ROI_AS;

        switch (request.grip)
        {
        case 0:
            grip_info.type = GRIP_TYPE::SDG;
            grip_info.cell_length = SDG_L;
            grip_info.cell_width  = SDG_W;
            ROS_INFO("Grip type: SDG");
            break;
        case 1:
            grip_info.type = GRIP_TYPE::SSG;
            grip_info.cell_length = SSG_L;
            grip_info.cell_width  = SSG_W;
            ROS_INFO("Grip type: SSG");
            break;
        case 2:
            grip_info.type = GRIP_TYPE::DDG;
            grip_info.cell_length = DDG_L;
            grip_info.cell_width  = DDG_W;
            ROS_INFO("Grip type: DDG");
            break;
        default:
            grip_info.type = GRIP_TYPE::DDG;
            grip_info.cell_length = DDG_L;
            grip_info.cell_width  = DDG_W;
            ROS_INFO("Grip type: DDG");
        }
        ROS_INFO("Robot position: %d-%d", request.robot_x, request.robot_y);
        ROS_INFO("ROI: %d %d %d %d", request.roi.x1, request.roi.y1, request.roi.x2, request.roi.y2);
    }

    bool serviceScanCommand(lidar_scan::ScanCommand::Request &req, lidar_scan::ScanCommand::Response &res)
    {
        ROS_INFO("Receive commamd from radio node");
        get_request(req);

        this->scan_start = true;

        ROS_INFO("Processing: %d", scan_done);
        res.response = return_value;
        return true;
    }

public:
    Lidar_Scan(char* ini_name, ros::NodeHandle *node): ini_file{ini_name}
    {
        get_config();
        cloud_pub   = node->advertise<sensor_msgs::PointCloud> ("pointcloud", 100);
        marker_pub  = node->advertise<visualization_msgs::Marker>("line_segment", 100);
        scan_sub    = node->subscribe("/scan", 1000, &Lidar_Scan::callback, this);
        server      = node->advertiseService("scan_command", &Lidar_Scan::serviceScanCommand, this);
    }

    ~Lidar_Scan()
    {
        iniparser_freedict(ini);
    }
};

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "node_process");
    ros::NodeHandle nh;
    Lidar_Scan process(argv[1], &nh);
    ros::spin();
    return 0;
}