#include <ros/ros.h>
#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>
#include <sun_robot_ros/RobotMotionClient.h>
#include <geometry_msgs/PoseArray.h>
#include "yaskawa_cross_modal/grid.h"
#include "sun_tactile_common/TactileStamped.h"
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/Float32MultiArray.h>
#include <Eigen/Dense>
#include <unsupported/Eigen/Polynomials>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include "yaskawa_cross_modal/grid.h"
#define foreach BOOST_FOREACH

template <typename T>
void bag_write(std::string path, std::string topic, T data) {

    rosbag::Bag bag;
    bag.open(path, rosbag::bagmode::Write);
    bag.write(topic,ros::Time::now(), data);
    bag.close();

}

template <typename T>
T bag_read(std::string path, std::string topic) {

    rosbag::Bag bag;
    bag.open(path, rosbag::bagmode::Read);
    rosbag::View view(bag, rosbag::TopicQuery(topic));
    T val;

    foreach(rosbag::MessageInstance const m, view)
    {
        boost::shared_ptr<T> pcl = m.instantiate<T>();
        if(pcl != NULL) {
            val = *pcl;
        }
    }

    bag.close();
    return val;

}

bool askContinue(const std::string &prompt = "")
{
    char ans;
    ros::Rate loop_rate(20);
    while(true)
    {
        std::cout << prompt << " - Press y/Y to continue [s/S to skip]: ";
        std::cin >> ans;
        if (ans == 'y' || ans == 'Y') return true;
        else if (ans == 's' || ans == 'S') return false;
        else std::cout << "Valore non ammesso, riprova" << std::endl;
        loop_rate.sleep();
    }

    throw std::runtime_error("USER STOP!");
}

std::vector<Eigen::MatrixXd> grad(Eigen::MatrixXd dS)
{

    std::vector<Eigen::MatrixXd> G;
    Eigen::MatrixXd dX, dY;
    dX.resize(dS.rows(), dS.cols());
    dY.resize(dS.rows(), dS.cols());

    for(int i = 0; i < dS.rows(); i++)
    {
        for(int j = 1; j < dS.cols()-1; j++)
        {
            dX(i,j) = 0.5*(dS(i,j+1) - dS(i,j-1));
        }
        dX(i,0) = dS(i,1) - dS(i,0);
        dX(i,dS.cols()-1) = dS(i,dS.cols()-1) - dS(i,dS.cols()-2);
    }

    for(int j = 0; j < dS.cols(); j++)
    {
        for(int i = 1; i < dS.rows()-1; i++)
        {
            dY(i,j) = 0.5*(dS(i+1,j) - dS(i-1,j));
        }
        dY(0,j) = dS(1,j) - dS(0,j);
        dY(dS.rows()-1,j) = dS(dS.rows()-1,j) - dS(dS.rows()-2,j);
    }

    G.push_back(dX);
    G.push_back(dY);
    return G;    
}

// Variabili globali
bool toccato;
std::vector<float> delta_v(12);
std::vector<float> sensor_data(12);
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

int main(int argc, char** argv)
{

    ros::init(argc, argv, "tactile_hessian");
    ros::NodeHandle nh;
    sun::RobotMotionClient robot(ros::NodeHandle(nh, "motoman"));
    robot.waitForServers();

    ros::Subscriber sub_volt = nh.subscribe("/tactile_voltage/rect", 1, tactile_delta_cb);
    ros::Subscriber sub_fkine = nh.subscribe("/motoman/clik/fkine", 1, fkine_cb);
    ros::Publisher pub_volt = nh.advertise<sun_tactile_common::TactileStamped>("/measure", 1);
    ros::Publisher pcl_centr_pub = nh.advertise<sensor_msgs::PointCloud>("/pcl2", 1);
    ros::Publisher centr_pub = nh.advertise<sensor_msgs::PointCloud>("/centroidi", 1);
    yaskawa_cross_modal::grid srv;
    geometry_msgs::PoseArray grid;
    ros::ServiceClient grid_client = nh.serviceClient<yaskawa_cross_modal::grid>("grid_srv");

    geometry_msgs::Pose end_effector, finger_reference;
    int rows = 6, cols = 2;
    double delta = 0.001;
    double z_limite = 0.240, quota = 0.0, z_start = 0.270;
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
    double x_min = -0.17, x_max = 0.125; 
    double y_min = -0.53, y_max = -0.30;
    geometry_msgs::Pose posa, start;
    Eigen::Quaterniond Q(0, 0.7071, 0.7071, 0);
    Q.normalize();
    // posa.position.x = -0.10;
    // posa.position.y = -0.40;
    // posa.position.z = 0.268; 
    posa.orientation.w = Q.w();
    posa.orientation.x = Q.x();
    posa.orientation.y = Q.y();
    posa.orientation.z = Q.z();

    start.position.x = x_min;
    start.position.y = y_max;
    start.position.z = z_start; 
    start.orientation.w = Q.w();
    start.orientation.x = Q.x();
    start.orientation.y = Q.y();
    start.orientation.z = Q.z();
    posa.position = start.position;

    if(askContinue("Home")) {
        robot.goTo(q0, ros::Duration(15.0));
        ROS_INFO_STREAM("Posa di Home raggiunta");
    }
    if(askContinue("Start")) {
        robot.goTo(start, ros::Duration(15.0));
    }

    double tempo = 0.5;
    double div, dmin = 0.015;
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
        grid.poses[i].position.z = z_start;

    for(int i = 1; i < divx; i = i+2)
        std::reverse(grid.poses.begin() + i*divy, grid.poses.begin() + (i+1)*divy);

    sensor_msgs::PointCloud centroide;
    centroide.header.frame_id = "base_link";
    if(askContinue("Avviare il ciclo di esplorazione?"))
    {
        for(int i = 0; i < grid.poses.size(); i++)
        {
            posa = grid.poses[i];
            while(ros::ok() && posa.position.z > z_limite)
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
        centroide.header.stamp = ros::Time::now();
        centr_pub.publish(centroide);
    }

    return 0;
}