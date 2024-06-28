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

bool toccato;
std::vector<float> sensor_data(12);
void tactile_cb(const sun_tactile_common::TactileStamped& msg) {
    sensor_data = msg.tactile.data;
	double sum = 0;
	sum = sensor_data[4] + sensor_data[7] + sensor_data[8];
	sensor_data[6] = sum/3;
    
    toccato = false;
    for(int i = 0; i < sensor_data.size(); i++)
    {
        if(sensor_data.at(i) > 0.1)
        {
            toccato = true;
            break;
        }
    }
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


int main(int argc, char** argv)
{

    ros::init(argc,argv,"tactile_map");
    ros::NodeHandle nh;
    sun::RobotMotionClient robot(ros::NodeHandle(nh, "motoman"));
    robot.waitForServers();

    ros::Subscriber sub_volt = nh.subscribe("/tactile_voltage/rect", 1, tactile_cb);
    ros::Publisher pub_volt = nh.advertise<sun_tactile_common::TactileStamped>("/measure", 1);

    geometry_msgs::Pose end_effector, finger_reference;
    int rows = 6, cols = 2;
    double delta = 0.002;
    double z_limite = 0.240;
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
    // finger_reference.position.x = -0.011;
    // finger_reference.position.y = -0.005;
    // finger_reference.position.z = 0.005;
    // Eigen::Quaterniond quat(1,0,0,0);
    // quat.normalize();
    // finger_reference.orientation.w = quat.w();
    // finger_reference.orientation.x = quat.x();
    // finger_reference.orientation.y = quat.y();
    // finger_reference.orientation.z = quat.z();
    robot.clik_.set_end_effector(end_effector);
    ros::Rate loop_rate(30.0);


    if(askContinue("Home")) {
        robot.goTo(q0, ros::Duration(15.0));
        ROS_INFO_STREAM("Posa di Home raggiunta");
    }

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

    // Posa di esplorazione
    geometry_msgs::Pose posa;
    Eigen::Quaterniond Q(0, 0.7071, 0.7071, 0);
    Q.normalize();
    posa.position.x = -0.10;
    posa.position.y = -0.40;
    posa.position.z = 0.268; 
    posa.orientation.w = Q.w();
    posa.orientation.x = Q.x();
    posa.orientation.y = Q.y();
    posa.orientation.z = Q.z();


    if(askContinue("Posa di partenza")) {
        robot.goTo(posa, ros::Duration(15));
        ROS_INFO_STREAM("Posa di Partenza Raggiunta");
    }

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
    // std::cout << coeff << std::endl << std::endl;
    // std::cout << Eigen::poly_eval(coeff.row(0).reverse(), 0.2);

    if(askContinue("Avviare il ciclo di esplorazione?"))
    {
        while(ros::ok() && posa.position.z > z_limite && exit == false)
        {
            posa.position.z -= delta;
            robot.goTo(posa, ros::Duration(2.0));
            ros::spinOnce();
            if(toccato)
            {
                posa.position.z -= 0.002;
                robot.goTo(posa, ros::Duration(2.0));
                ros::spinOnce();
                std::cout << "Toccato" << std::endl;

                for(int i = 0; i < p_si.size(); i++)
                    p_si.at(i).z() = sensor_data.at(i);
                
                for(int i = 0; i < p_si.size(); i++)
                {
                    mappa(i,0) = p_si.at(i).x();
                    mappa(i,1) = p_si.at(i).y();
                    mappa(i,2) = p_si.at(i).z();
                }
                std::cout << "Mappa Tattile: " << std::endl << mappa << std::endl << std::endl;

                sun_tactile_common::TactileStamped msg_volt;
                msg_volt.tactile.data = sensor_data;
                msg_volt.header.stamp = ros::Time::now();
                pub_volt.publish(msg_volt);

                /* Applicazione dell'equalizzazione */
                for(int i = 0; i < coeff.rows(); i++)
                {  
                    // std::cout << "Tensione di ingresso: " << p_si.at(i).z() << std::endl;
                    p_si.at(i).z() = Eigen::poly_eval(coeff.row(i).reverse(), p_si.at(i).z());
                    // std::cout << "Displacement in uscita (equalizzato): " << p_si.at(i).z() << std::endl;
                }

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
                std::cout << dS << std::endl << std::endl << std::endl;

                std::vector<Eigen::MatrixXd> G = grad(dS);
                Eigen::MatrixXd gx = G.at(0);
                Eigen::MatrixXd gy = G.at(1);

                // std::cout << "Gradiente lungo x:" << std::endl << G.at(0) << std::endl << std::endl;
                // std::cout << "Gradiente lungo y:" << std::endl << G.at(1) << std::endl << std::endl;
                
                std::vector<Eigen::MatrixXd> GX = grad(gx);
                std::vector<Eigen::MatrixXd> GY = grad(gy);

                Eigen::MatrixXd gxx = GX.at(0);
                Eigen::MatrixXd gxy = GX.at(1);
                Eigen::MatrixXd gyx = GY.at(0);
                Eigen::MatrixXd gyy = GY.at(1);

                // std::cout << "Gradiente lungo xx:" << std::endl << gxx << std::endl << std::endl;
                // std::cout << "Gradiente lungo xy:" << std::endl << gxy << std::endl << std::endl;
                // std::cout << "Gradiente lungo yx:" << std::endl << gyx << std::endl << std::endl;
                // std::cout << "Gradiente lungo yy:" << std::endl << gyy << std::endl << std::endl;

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


                std::cout << nH << std::endl << std::endl;
                std::cout << "INDICATORE: " << nH.norm() << std::endl;

                if(askContinue("Continuare?")) { posa.position.z = 0.268; robot.goTo(posa, ros::Duration(4.0)); }
                else exit = true;
            }
        }
    }

    return 0;
}