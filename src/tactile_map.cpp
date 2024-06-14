#include <ros/ros.h>
#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>
#include <sun_robot_ros/RobotMotionClient.h>
#include <geometry_msgs/PoseArray.h>
#include "yaskawa_cross_modal/grid.h"
#include "sun_tactile_common/TactileStamped.h"
#include <sensor_msgs/PointCloud.h>
#include <Eigen/Dense>
#include <TooN/TooN.h>

bool toccato;
std::vector<float> sensor_data(12);
void getSensorData(const sun_tactile_common::TactileStamped& msg) {
    sensor_data = msg.tactile.data;
	// sensor_data[6] = sensor_data[4];
	// sensor_data[6] = sensor_data[7];
	// sensor_data[6] = sensor_data[8];
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
    while(true)
    {
        std::cout << prompt << " - Press y/Y to continue [s/S to skip]: ";
        std::cin >> ans;
        if (ans == 'y' || ans == 'Y') return true;
        else if (ans == 's' || ans == 'S') return false;
        else std::cout << "Valore non ammesso, riprova" << std::endl;
    }

    throw std::runtime_error("USER STOP!");
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"tactile_map");
    ros::NodeHandle nh;
    sun::RobotMotionClient robot(ros::NodeHandle(nh, "motoman"));
    // robot.waitForServers();

    ros::Subscriber sub_volt = nh.subscribe("/tactile_voltage/rect", 1, getSensorData);
    ros::Publisher pub_volt = nh.advertise<sun_tactile_common::TactileStamped>("/measure", 1);
    geometry_msgs::Pose end_effector, finger_reference;
    int rows = 6, cols = 2;
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

    // // Generazione delle coordinate (xi,yi) in terna reference taxels
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

    if(askContinue("Avviare il movimento?"))
    {
        geometry_msgs::Pose posa;
        TooN::Vector<2> h_sum;
        TooN::Vector<6> v_sum;
        TooN::Vector<2> h_vero;
        TooN::Vector<6> v_vero;
        TooN::Matrix<6,2> delta_v;

        // Posa di partenza, arrivando dalla posa di Home q0
        Eigen::Quaterniond Q(0, 0.7071, 0.7071, 0);
        Q.normalize();
        posa.position.x = -0.10;
        posa.position.y = -0.40;
        posa.position.z = 0.26; 
        posa.orientation.w = Q.w();
        posa.orientation.x = Q.x();
        posa.orientation.y = Q.y();
        posa.orientation.z = Q.z();

        while(ros::ok())
        {
            
            robot.goTo(posa, ros::Duration(12.0));
            ros::spinOnce();
            if(toccato)
            {

                int cnt = 0;
                for(int i = 0; i < rows; i++)
                {
                    for(int j = 0; j < cols; j++)
                    {
                        delta_v(i,j) = sensor_data[cnt];
                        cnt++;
                    }
                }

                // Detection of the main direction
                h_sum = TooN::Zeros;
                v_sum = TooN::Zeros;
                h_vero = TooN::Zeros;
                v_vero = TooN::Zeros;
                double h = 0.0, v = 0.0;

                for(int i = 0; i < rows; i++)
                    h_vero += delta_v[i];
                h_sum = h_vero/rows;

                for(int i = 0; i < cols; i++)
                    v_vero += delta_v.T()[i];
                v_sum = v_vero/cols;
            
                h = h_sum[0];
                v = v_sum[0];
                // std::cout << "h: " << h << std::endl;
                // std::cout << "v: " << v << std::endl;

                for(int i = 1; i < cols; i++)
                    if(h_sum[i] < h) h = h_sum[i];							

                for(int i = 1; i < rows; i++)
                    if(v_sum[i] < v) v = v_sum[i];

                if(h>v) std::cout << "Orizzontale" << std::endl;
                else std::cout << "Verticale" << std::endl;

                for(int i = 0; i < p_si.size(); i++)
				    p_si.at(i).z() = sensor_data.at(i);

                for(int i = 0; i < p_si.size(); i++)
                {
                    mappa(i,0) = p_si.at(i).x();
                    mappa(i,1) = p_si.at(i).y();
                    mappa(i,2) = p_si.at(i).z();
                }

                std::cout << "Mappa Tattile: " << std::endl << mappa << std::endl;

                // SVD Eigen
                // double media = 0;
                // for(int i = 0; i < mappa.row(0).size(); i++)
                // {
                //     media = mappa.col(i).mean();
                //     std::cout << "Media colonna " << i << ": " << media << std::endl;
                //     for(int j = 0; j < mappa.col(0).size(); j++)
                //         mappa(j,i) = mappa(j,i) - media;
                // }

                // std::cout << "Mappa Tattile Normalizzata: " << std::endl << mappa << std::endl;

                // Eigen::JacobiSVD<Eigen::MatrixXd> svd(mappa, Eigen::ComputeThinU | Eigen::ComputeThinV);
                // Eigen::MatrixXd U = svd.matrixU();
                // Eigen::MatrixXd V = svd.matrixV();
                // Eigen::VectorXd S = svd.singularValues();
                // std::cout << "Valori Singolari: "<< std::endl << S << std::endl;
 
                // for(int i = 0; i < sensor_data.size(); i++)
                //     std::cout << sensor_data.at(i) << std::endl;

                // count = 1;
                // for i = 1:size(tempor)/2
                //     Zor(i,:) = tempor(count:count+1);
                //     Zvert(i,:) = tempver(count:count+1);
                //     Ztav(i,:) = temptav(count:count+1);
                //     count = count + 2;
                // end

                // Eigen::MatrixXd dV;
                // dV.resize(rows,cols);
                // int count = 0;
                // for(int i = 0; i < rows/2; i++)
                //     for(int j = 0; j < cols/2; j++)
                //     {
                //         dV(i,j) = p_si.at(count).z();
                //         count += 1;
                //     }
                
                

                sun_tactile_common::TactileStamped msg_volt;
                msg_volt.tactile.data = sensor_data;
                msg_volt.header.stamp = ros::Time::now();
                pub_volt.publish(msg_volt);
                posa.position.z += 0.015;
                robot.goTo(posa, ros::Duration(4.0));
                if(askContinue("Continuare?")) {}
                else break;
                
            }
            posa.position.z -= 0.004;
            toccato = false;
            loop_rate.sleep();
        }
    }

    return 0;

}