#include <ros/ros.h>
#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>
#include <sun_robot_ros/RobotMotionClient.h>
#include <geometry_msgs/PoseArray.h>
#include <actionlib/server/simple_action_server.h>
#include "yaskawa_cross_modal/grid.h"
#include "sun_tactile_common/TactileStamped.h"
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/Float32MultiArray.h>
#include <Eigen/Dense>
#include <unsupported/Eigen/Polynomials>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include "yaskawa_cross_modal/grid.h"
#include "yaskawa_cross_modal/utility.h"
#include "yaskawa_cross_modal/TactileAction.h"

// Variabili globali
bool toccato;
std::vector<float> delta_v(12);
geometry_msgs::PoseStamped base_finger;

// Callbacks
void tactile_delta_cb(const sun_tactile_common::TactileStamped& msg) {
    delta_v = msg.tactile.data;
	double sum = 0;
	sum = delta_v[4] + delta_v[7] + delta_v[8];
	delta_v[6] = sum/3;
    
    toccato = false;
    for(int i = 0; i < delta_v.size(); i++)
    {
        if(delta_v.at(i) > 0.1)
        {
            toccato = true;
            break;
        }
    }
}

void fkine_cb(const geometry_msgs::PoseStampedPtr &msg) {
    
    base_finger = *msg;
    Eigen::Quaterniond q(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    q.normalize();
    base_finger.pose.orientation.w = q.w();
    base_finger.pose.orientation.x = q.x();
    base_finger.pose.orientation.y = q.y();
    base_finger.pose.orientation.z = q.z();
    
}

class TactileActionServer
{
public:

    ros::NodeHandle nh;
    actionlib::SimpleActionServer<yaskawa_cross_modal::TactileAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
    std::string action_name_;
    // create messages that are used to published feedback/result
    // yaskawa_cross_modal::task_tactileAction::feedback_;
    yaskawa_cross_modal::TactileResult result_;

    TactileActionServer(std::string name) : as_(nh, name, boost::bind(&TactileActionServer::executeCB, this, _1), false)
    {
        as_.start();
    }

    ~TactileActionServer() = default;

    void executeCB(const yaskawa_cross_modal::TactileGoalConstPtr &goal)
    {

        sun::RobotMotionClient robot(ros::NodeHandle(nh, "motoman"));
        robot.waitForServers();
        bool success = true;

        ros::Subscriber sub_volt = nh.subscribe("/tactile_voltage/rect", 1, tactile_delta_cb);
        ros::Subscriber sub_fkine = nh.subscribe("/motoman/clik/fkine", 1, fkine_cb);
        ros::Publisher pub_volt = nh.advertise<sun_tactile_common::TactileStamped>("/measure", 1);
        ros::Publisher pcl_centr_pub = nh.advertise<sensor_msgs::PointCloud>("/pcl2", 1);
        ros::Publisher centr_pub = nh.advertise<sensor_msgs::PointCloud>("/tactile_cloud", 1);
        yaskawa_cross_modal::grid srv;
        geometry_msgs::PoseArray grid;
        ros::ServiceClient grid_client = nh.serviceClient<yaskawa_cross_modal::grid>("grid_srv");

        geometry_msgs::Pose end_effector, finger_reference;
        int rows = goal->rows, cols = goal->cols;
        double z_start = goal->start.position.z;
        double delta = 0.001;
        double z_limite = 0.240, quota = 0.0;
        bool exit = false;
        const std::vector<double> q0 = {-1.6097532510757446, -0.34197139739990234, 0.0951523706316948, -0.42233070731163025, -0.016888976097106934, -1.4525935649871826, 0.03369557857513428};
        Eigen::MatrixXd mappa;
        mappa.resize(12,3);

        // Posa per settare l'end effector, ottenuta dalla trasformata tra tool0 e finger
        end_effector.position.x = 0.025;
        end_effector.position.y = 0.065;
        end_effector.position.z = 0.041;
        Eigen::Quaterniond Quat(0.707107, -2.25214e-06, 2.25214e-06, -0.707107);
        Quat.normalize();
        end_effector.orientation.x = Quat.x();
        end_effector.orientation.y = Quat.y();
        end_effector.orientation.z = Quat.z();
        end_effector.orientation.w = Quat.w();

        // Posa del pad del sensore in terna finger
        finger_reference.position.x = -0.011;
        finger_reference.position.y = -0.005;
        finger_reference.position.z = 0.005;
        Eigen::Quaterniond quat(1,0,0,0);
        quat.normalize();
        finger_reference.orientation.w = quat.w();
        finger_reference.orientation.x = quat.x();
        finger_reference.orientation.y = quat.y();
        finger_reference.orientation.z = quat.z();
        robot.clik_.set_end_effector(end_effector);
        ros::Rate loop_rate(30.0);

        // // Generazione delle coordinate (xi,yi) in terna reference taxel
        std::vector<Eigen::Vector3d> p_si;
        {
            int count = 0;
            double refx = 0;
            double refy = 0;
            double offset = 0.0035;

            for(int j = 0; j < rows; j++) {
                refy = 0;
                for(int k = 0; k < cols; k++) {
                    Eigen::Vector3d temp(refx,refy,0);
                    p_si.push_back(temp);
                    refy = refy + offset;
                    count++;
                }
                refx = refx + offset;
            }  
        }    

        // Pose di esplorazione
        double x_min = goal->x_min, x_max = goal->x_max; 
        double y_min = goal->y_min, y_max = goal->y_max;
        double dmin = goal->dmin;
        Eigen::Quaterniond Q(goal->start.orientation.w, goal->start.orientation.x, goal->start.orientation.y, goal->start.orientation.z);
        geometry_msgs::Pose start = goal->start;

        geometry_msgs::Pose posa;
        Q.normalize();

        start.position.x = x_min;
        start.position.y = y_max;
        start.position.z = z_start; 
        start.orientation.w = Q.w();
        start.orientation.x = Q.x();
        start.orientation.y = Q.y();
        start.orientation.z = Q.z();

        if(askContinue("Home")) {
            robot.goTo(q0, ros::Duration(15.0));
            ROS_INFO_STREAM("Posa di Home raggiunta");
        }

        if(askContinue("Start")) {
            robot.goTo(start, ros::Duration(15.0));
        }

        double tempo = 0.5;
        double div;
        int divx, divy;

        div = (x_max-x_min)/dmin;
        if(div < 1) divx = 1;
        else divx = ceil(div); /*divx = floor(div)*/

        div = (y_max-y_min)/dmin;
        if(div < 1) divy = 1;
        else divy = ceil(div);

        std::cout << "DIVX: " << divx << std::endl;
        std::cout << "DIVY: " << divy << std::endl;

        // Creazione della griglia tramite il servizio
        srv.request.w = start.orientation.w;
        srv.request.x = start.orientation.x;
        srv.request.y = start.orientation.y;
        srv.request.z = start.orientation.z;
        srv.request.quota = quota;
        srv.request.x0 = start.position.x;
        srv.request.xf = x_max;
        srv.request.y0 = start.position.y;
        srv.request.yf = y_min;
        srv.request.divx = divx;
        srv.request.divy = divy;
        if(grid_client.call(srv)) { grid = srv.response.grid; ROS_INFO("Griglia ottenuta"); }
        else { ROS_INFO("Errore nella generazione della griglia"); }

        std_msgs::Float32MultiArray coefficiente;
        coefficiente = bag_read<std_msgs::Float32MultiArray>("/home/workstation2/ws_cross_modal/bags/coefficienti.bag", "/coefficienti");
        //std::cout << coefficiente << std::endl;
        Eigen::MatrixXd coeff;
        coeff.resize(12,3);

        int cont = 0;
        for(int i = 0; i < coeff.cols(); i++)
        {
            for(int j = 0; j < coeff.rows(); j++)
            {
                coeff(j,i) = coefficiente.data.at(cont);
                cont++;
            }
        }

        for(int i = 0; i < grid.poses.size(); i++)
        {
            grid.poses[i].position.z = z_start;
            std::cout << grid.poses[i] << std::endl << std::endl;
        }

        for(int i = 1; i < divx; i = i+2)
            std::reverse(grid.poses.begin() + i*divy, grid.poses.begin() + (i+1)*divy);

        sensor_msgs::PointCloud centroide;
        centroide.header.frame_id = "base_link";
        if(askContinue("Avviare il ciclo di esplorazione?"))
        {
            for(int i = 0; i < grid.poses.size(); i++)
            {

                if (as_.isPreemptRequested() || !ros::ok())
                {
                    ROS_INFO("%s: Preempted", action_name_.c_str());
                    // set the action state to preempted
                    as_.setPreempted();
                    success = false;
                    break;
                }

                posa = grid.poses[i];
                exit = false;
                while(ros::ok() && posa.position.z > z_limite && exit == false)
                {
    
                    posa.position.z -= delta;
                    robot.goTo(posa, ros::Duration(tempo)); 
                    ROS_INFO_STREAM("Posa raggiunta");

                    ros::spinOnce();
                    if(toccato)
                    {
                        posa.position.z -= delta;
                        robot.goTo(posa, ros::Duration(tempo));
                        ros::spinOnce();
                        std::cout << "Toccato" << std::endl;

                        for(int i = 0; i < p_si.size(); i++)
                            p_si.at(i).z() = delta_v.at(i);
                        
                        for(int i = 0; i < p_si.size(); i++)
                        {
                            mappa(i,0) = p_si.at(i).x();
                            mappa(i,1) = p_si.at(i).y();
                            mappa(i,2) = p_si.at(i).z();
                        }
                        std::cout << "Mappa Tattile: " << std::endl << mappa << std::endl << std::endl;

                        sun_tactile_common::TactileStamped msg_volt;
                        msg_volt.tactile.data = delta_v;
                        msg_volt.header.stamp = ros::Time::now();
                        pub_volt.publish(msg_volt);

                        /* Applicazione dell'equalizzazione */
                        for(int i = 0; i < coeff.rows(); i++)
                            p_si.at(i).z() = Eigen::poly_eval(coeff.row(i).reverse(), p_si.at(i).z());

                        Eigen::MatrixXd dS;
                        dS.resize(rows,cols);
                        int count = 0;
                        for(int i = 0; i < rows; i++)
                        {
                            for(int j = 0; j < cols; j++)
                            {
                                dS(i,j) = p_si.at(count).z();
                                count += 1;
                            }
                        }

                        std::vector<Eigen::MatrixXd> G = grad(dS);
                        Eigen::MatrixXd gx = G.at(0);
                        Eigen::MatrixXd gy = G.at(1);
                        
                        std::vector<Eigen::MatrixXd> GX = grad(gx);
                        std::vector<Eigen::MatrixXd> GY = grad(gy);

                        Eigen::MatrixXd gxx = GX.at(0);
                        Eigen::MatrixXd gxy = GX.at(1);
                        Eigen::MatrixXd gyx = GY.at(0);
                        Eigen::MatrixXd gyy = GY.at(1);

                        Eigen::MatrixXd nH;
                        nH.resize(rows, cols);

                        for(int i = 0; i < rows; i++)
                        {
                            for(int j = 0; j < cols; j++)
                            {
                                Eigen::MatrixXd temp;
                                temp.resize(G.size(), G.size());
                                temp.row(0) << gxx(i,j), gxy(i,j);
                                temp.row(1) << gyx(i,j), gyy(i,j);
                                nH(i,j) = temp.norm();
                            }
                        }

                        double indicatore = nH.norm();

                        std::cout << nH << std::endl << std::endl;
                        std::cout << "INDICATORE: " << indicatore << std::endl;

                        if(indicatore > 0.0009)
                        {
                            std::cout << "CAVO" << std::endl;
                            std::cout << posa.position;
                            Eigen::Matrix4d T_base_reference, T_base_finger, T_finger_reference;
                            Eigen::Quaterniond q_finger_reference(finger_reference.orientation.w, finger_reference.orientation.x, finger_reference.orientation.y, finger_reference.orientation.z);
                            q_finger_reference.normalize();
                            Eigen::Vector3d p_finger_reference(finger_reference.position.x, finger_reference.position.y, finger_reference.position.z);
                            Eigen::Matrix3d R_finger_reference = q_finger_reference.toRotationMatrix();
                            Eigen::Quaterniond q_base_finger(base_finger.pose.orientation.w, base_finger.pose.orientation.x, base_finger.pose.orientation.y, base_finger.pose.orientation.z);
                            q_base_finger.normalize();
                            Eigen::Vector3d p_base_finger(base_finger.pose.position.x, base_finger.pose.position.y, base_finger.pose.position.z);
                            Eigen::Matrix3d R_base_finger = q_base_finger.toRotationMatrix();
                            Eigen::Vector3d o_b_s = p_base_finger + p_finger_reference;
                            Eigen::Matrix3d R_b_s = R_base_finger * R_finger_reference;
                            Eigen::Vector3d p_b_s, p_b_s_centr;
                            double sumx = 0.0, sumy = 0.0, sumv = 0.0;
                            
                            // Calcolo numeratore e denominatore della formula del centroide
                            for(int i = 0; i < delta_v.size(); i++) 
                            {
                                sumv += delta_v[i];
                                sumx += p_si[i].x()*delta_v[i];
                                sumy += p_si[i].y()*delta_v[i];
                            }

                            geometry_msgs::Point32 punto;
                            punto.x = sumx/sumv;
                            punto.y = sumy/sumv;
                            punto.z = 0;
                            std::cout << punto << std::endl;
                            Eigen::Vector3d point(punto.x, punto.y, punto.z);
                            p_b_s_centr = o_b_s + R_b_s * point;
                            punto.x = p_b_s_centr.x();
                            punto.y = p_b_s_centr.y();
                            punto.z = p_b_s_centr.z();
                            centroide.points.push_back(punto);
                            centroide.header.stamp = ros::Time::now();
                            pcl_centr_pub.publish(centroide);
                            std::cout << "Centroide appena calcolato:" << std::endl << punto << std::endl;
                        }
                        else
                        {
                            std::cout << "TAVOLO" << std::endl;
                        }

                        posa = grid.poses[i];
                        robot.goTo(posa, ros::Duration(tempo));
                        exit = true;
                    }
                }
                loop_rate.sleep();
            }
            if(success)
            {
                centroide.header.stamp = ros::Time::now();
                centr_pub.publish(centroide);
                ROS_INFO("%s: Succeeded", action_name_.c_str());
                as_.setSucceeded(result_);
            }
        }
    }
};

int main(int argc, char** argv)
{

    ros::init(argc, argv, "tactile_exploration");
    TactileActionServer server("tactile_action");
    ros::spin();
    
    return 0;
}