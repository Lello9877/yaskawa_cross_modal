#include <ros/ros.h>
#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>
#include <sun_robot_ros/RobotMotionClient.h>
#include <geometry_msgs/PoseArray.h>
#include "yaskawa_cross_modal/grid.h"
#include "sun_tactile_common/TactileStamped.h"
#include <sensor_msgs/PointCloud.h>
#include <Eigen/Dense>

// Variabili globali per gestire il tocco del sensore, le tensioni e la posa in cui avviene il tocco
bool touched = false;
sun_tactile_common::TactileStamped tensione;
geometry_msgs::PoseStamped base_finger;

bool askContinue(const std::string &prompt = "")
{
    char ans;
    std::cout << prompt << " - Press y to continue [s to skip]: ";
    std::cin >> ans;

    if (ans == 'y' || ans == 'Y') return true;
    else if (ans == 's' || ans == 'S') return false;
    else return false;

    throw std::runtime_error("USER STOP!");
}

// Callback per prelevare il valore delle tensioni
void tact_cb(const sun_tactile_common::TactileStampedPtr &msg) {

    // Somma delle tensioni senza contatto: 13.91, 13.85
    double sum = 0, treshold = 13.99;
    double v_min[msg->tactile.data.size()] = {1, 1.04, 1, 1.06, 1, 0.95, 3.02, 0.90, 0.96, 1, 1.11, 0.86};

    for(int i = 0; i < msg->tactile.data.size(); i++) {
        sum = sum + msg->tactile.data[i];
        msg->tactile.data[i] = abs(msg->tactile.data[i] - v_min[i]);
    }
    
    tensione = *msg;

    if(sum <= treshold) touched = false; 
    else touched = true;

}

// Callback per prelevare la posa dell'end effector
void fkine_cb(const geometry_msgs::PoseStampedPtr &msg) {
    
    base_finger = *msg;
    Eigen::Quaterniond q(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    q.normalize();
    base_finger.pose.orientation.w = q.w();
    base_finger.pose.orientation.x = q.x();
    base_finger.pose.orientation.y = q.y();
    base_finger.pose.orientation.z = q.z();
    
}

int main(int argc, char *argv[])
{
    // Inizializzazione del nodo e dell'oggetto robot su cui chiamare goTo in giunti o cartesiano
    ros::init(argc,argv,"task");
    ros::NodeHandle nh;
    sun::RobotMotionClient robot(ros::NodeHandle(nh, "motoman"));
    robot.waitForServers();

    // Definizione dei publisher, subscriber, servizio di creazione griglia
    geometry_msgs::Pose start, end_effector, finger_reference;
    geometry_msgs::PoseArray grid;
    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud>("/pcl",1);
    ros::Publisher pcl_centr_pub = nh.advertise<sensor_msgs::PointCloud>("/pcl2",1);
    ros::Subscriber sub_volt = nh.subscribe("/tactile_voltage", 1, tact_cb);
    ros::Subscriber sub_fkine = nh.subscribe("/motoman/clik/fkine", 1, fkine_cb);
    ros::ServiceClient grid_client = nh.serviceClient<yaskawa_cross_modal::grid>("grid_srv");
    yaskawa_cross_modal::grid srv;
    Eigen::Quaterniond Q(0, 0.7071, 0.7071, 0);

    // Posa di partenza, arrivando dalla posa di Home q0
    Q.normalize();
    start.position.x = -0.195;
    start.position.y = -0.40;
    start.position.z = 0.29; 
    start.orientation.w = Q.w();
    start.orientation.x = Q.x();
    start.orientation.y = Q.y();
    start.orientation.z = Q.z();
    const std::vector<double> q0 = {-1.6097532510757446, -0.34197139739990234, 0.0951523706316948, -0.42233070731163025, -0.016888976097106934, -1.4525935649871826, 0.03369557857513428};

    // tf2_ros::Buffer tfBuffer;
    // tf2_ros::TransformListener tfListener(tfBuffer);
    // ros::Rate loop_rate(50.0);

    // geometry_msgs::TransformStamped transformStamped;
    // try {
    //     transformStamped = tfBuffer.lookupTransform("tool0", "finger", ros::Time(0), ros::Duration(3.0)); 
    // }
    // catch (tf2::TransformException &ex) {
    //     ROS_WARN("%s",ex.what());
    //     ros::Duration(1.0).sleep();
    // }
    // std::cout << transformStamped << std::endl;

    // geometry_msgs::TransformStamped transformStamped;
    // try {
    //     transformStamped = tfBuffer.lookupTransform("finger", "reference_taxel", ros::Time(0), ros::Duration(3.0)); 
    // }
    // catch (tf2::TransformException &ex) {
    //     ROS_WARN("%s",ex.what());
    //     ros::Duration(1.0).sleep();
    // }
    // std::cout << transformStamped << std::endl;

    // // Prelevo la posa per settare l'end effector del robot
    // end_effector.orientation = transformStamped.transform.rotation;
    // end_effector.position.x = transformStamped.transform.translation.x;
    // end_effector.position.y = transformStamped.transform.translation.y;
    // end_effector.position.z = transformStamped.transform.translation.z;

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
    robot.clik_.set_end_effector(end_effector);
    
    ros::Rate loop_rate(30.0);

    // if(askContinue("Avvio?")) {}
    // for(int i = 0; i < 20; i++) {
    //     robot.goTo(start, ros::Duration(15.0));
    //     //sleep(1);
    //     robot.goTo(prova, ros::Duration(15.0));
    //     //sleep(1);
    //     loop_rate.sleep();
    // }

    if(askContinue("Home")) {
        robot.goTo(q0, ros::Duration(10.0));
        ROS_INFO_STREAM("Posa di Home raggiunta");
    }

    if(askContinue("Posizione utile")) {
        robot.goTo(start, ros::Duration(20.0));
        ROS_INFO_STREAM("Posa di partenza raggiunta");
    }

    // Creazione della griglia tramite il servizio
    srv.request.w = start.orientation.w;
    srv.request.x = start.orientation.x;
    srv.request.y = start.orientation.y;
    srv.request.z = start.orientation.z;
    srv.request.quota = 0;
    srv.request.x0 = -0.195;
    srv.request.xf = 0.195;
    srv.request.y0 = -0.40;
    srv.request.yf = -0.55;
    srv.request.divx = 15;
    srv.request.divy = 3;
    if(grid_client.call(srv)) { grid = srv.response.grid; ROS_INFO("Griglia ottenuta"); }
    else { ROS_INFO("Errore nella generazione della griglia"); }

    // Calcolo dei coefficienti k per ogni cella
    int dim = 12;
    int rows = 6;
    int cols = 2;
    double k[dim];
    {
        double h_min = -0.008, h_max = -0.006;
        double h = h_max - h_min;
        double v_max[dim] = {1.65, 1.58, 1.44, 1.41, 1.34, 1.13, 3.04, 1.07, 1.28, 1.17, 1.44, 1.03};
        double v_min[dim] = {1, 1.04, 1, 1.06, 1, 0.95, 3.02, 0.90, 0.96, 1, 1.11, 0.86};

        for(int i = 0; i < dim; i++)
            k[i] = h/(v_max[i]-v_min[i]);
            //std::cout << k[i] << std::endl;
    }

    // Generazione delle coordinate (xi,yi) in terna reference_taxel

    Eigen::Vector3d p_si[dim];
    {
        int count = 0;
        double refx = 0;
        double refy = 0;
        double offset = 0.0035;

        for(int j = 0; j < rows; j++) {
            refy = 0;
            for(int k = 0; k < cols; k++) {
                Eigen::Vector3d temp(refx,refy,0);
                p_si[count] = temp;
                refy = refy + offset;
                count++;
            }
            refx = refx + offset;
        }  
    }

    // Inizio esplorazione
    std::vector<geometry_msgs::Point32> PCL;
    std::vector<geometry_msgs::Point32> PCL2;
    geometry_msgs::Pose posa;
    sensor_msgs::PointCloud cluster;
    sensor_msgs::PointCloud centroide;
    cluster.header.frame_id = "base_link";
    centroide.header.frame_id = "base_link";
    double z, z0 = 0.269, deltaz = -0.0040;
    int contatore = 0;

    if(askContinue("Avviare l'esplorazione?")) {
        for(int i = 0; i < grid.poses.size(); i++) {
            touched = false;
            z = z0;
            posa = grid.poses[i];
            posa.position.z = z;
            while(ros::ok() && z > 0.25 && touched == false)
            {   
                robot.goTo(posa, ros::Duration(7.0)); ROS_INFO_STREAM("Posa raggiunta");
                /*if(askContinue("posa"))*/
                z = z + deltaz;
                posa.position.z = z;
                std::cout << z << std::endl;
                ros::spinOnce();
                if(touched) {

                    // geometry_msgs::TransformStamped tf_b_s;
                    // try {
                    //     tf_b_s = tfBuffer.lookupTransform("base_link", "reference_taxel", ros::Time(0), ros::Duration(3.0)); 
                    // }
                    // catch (tf2::TransformException &ex) {
                    //     ROS_WARN("%s",ex.what());
                    //     ros::Duration(1.0).sleep();
                    // }
                    
                    // Estrapolo l'origine della terna sensore di riferimento per le celle, rispetto alla terna base
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

                    std::vector<geometry_msgs::Point32> pcl;
                    pcl.resize(tensione.tactile.data.size());

                    // Calcolo delle quote z delle celle del sensore
                    for(int i = 0; i < tensione.tactile.data.size(); i++)
                        p_si[i].z() = -k[i]*tensione.tactile.data[i];

                    // Calcolo i punti da inserire nelle point cloud delle celle e del centroide
                    // Costruzione della point cloud della cella
                    Eigen::Vector3d p_b_s, p_b_s_centr;
                    float sumx = 0, sumy = 0, sumv = 0;
                    for(int i = 0; i < tensione.tactile.data.size(); i++) {
                        p_b_s = o_b_s + R_b_s*p_si[i];
                        pcl.at(i).x = p_b_s.x();
                        pcl.at(i).y = p_b_s.y();
                        pcl.at(i).z = p_b_s.z();
                        PCL.push_back(pcl.at(i));
                    }

                    for(int i = 0; i < tensione.tactile.data.size(); i++) {
                        cluster.points.push_back(PCL.at(contatore));
                        contatore++;
                    }

                    cluster.header.stamp = ros::Time::now();
                    pcl_pub.publish(cluster); 

                    // Calcolo numeratore e denominatore della formula del centroide
                    for(int i = 0; i < tensione.tactile.data.size(); i++) {
                        sumv = sumv + tensione.tactile.data[i];
                        sumx = sumx + p_si[i].x()*tensione.tactile.data[i];
                        sumy = sumy + p_si[i].y()*tensione.tactile.data[i];
                    }

                    // Costruzione della point cloud dei centroidi
                    geometry_msgs::Point32 punto;
                    punto.x = sumx/sumv;
                    punto.y = sumy/sumv;
                    punto.z = 0;
                    std::cout << punto << std::endl;
                    Eigen::Vector3d point(punto.x,punto.y,punto.z);
                    p_b_s_centr = o_b_s + R_b_s*point;
                    punto.x = p_b_s_centr.x();
                    punto.y = p_b_s_centr.y();
                    punto.z = p_b_s_centr.z();
                    centroide.points.push_back(punto);
                    centroide.header.stamp = ros::Time::now();
                    pcl_centr_pub.publish(centroide);

                } // fine dell'if del tocco

                loop_rate.sleep();

            }  // fine del while interno 

            int dim = PCL.size();
            std::cout << PCL.size() << std::endl;
            posa.position.z = posa.position.z - 5*deltaz;
            robot.goTo(posa, ros::Duration(8.0));

        }   // fine del for esterno
    } // fine esplorazione
    return 0;
}