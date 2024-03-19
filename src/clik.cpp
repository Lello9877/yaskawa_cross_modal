#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "std_srvs/SetBool.h"
#include <Eigen/Dense>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>


// Funzione per garantire la continuità del quaternione, il problema nasce dal fatto che
// si ha la stessa rotazione tra Q e -Q
Eigen::Quaterniond quaternionContinuity(const Eigen::Quaterniond &q, const Eigen::Quaterniond &oldQ)
{
    auto tmp = q.vec().transpose() * oldQ.vec();
    if (tmp < -0.01)
    {
        Eigen::Quaterniond out(q);
        out.vec() = -out.vec();
        out.w() = -out.w();
        return out;
    }
    return q;
}

// Variabili globali per: vedere se il clik è attivo o meno, per sapere lo "stato del clik", e
// per la contiuità del quaternione
bool b_clik_run = false;                                           
moveit::core::RobotStatePtr kinematic_state;                       
Eigen::Quaterniond oldQuaternion = Eigen::Quaterniond::Identity();

// Callbk per la lettura della posa desiderata
// Nota l'utilizzo delle variabili globali per "comunicare" con il ciclo pricipale la posa desiderata.

Eigen::Vector3d position_des;
Eigen::Quaterniond quaternion_des;
void pose_des_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    position_des(0) = msg->pose.position.x;
    position_des(1) = msg->pose.position.y;
    position_des(2) = msg->pose.position.z;

    quaternion_des.x() = msg->pose.orientation.x;
    quaternion_des.y() = msg->pose.orientation.y;
    quaternion_des.z() = msg->pose.orientation.z;
    quaternion_des.w() = msg->pose.orientation.w;

    // assicuro che il quaternione che ricevo come desiderato è "continuo" 
    // con lo stato attuale del robot nel nodo clik
    quaternion_des = quaternionContinuity(quaternion_des, oldQuaternion);
}


// Callback per la lettura del twist 
/*Eigen::Matrix<double, 6, 1> b_vel_des;
void twist_des_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    b_vel_des(0) = msg->twist.linear.x;
    b_vel_des(1) = msg->twist.linear.y;
    b_vel_des(2) = msg->twist.linear.z;
    b_vel_des(3) = msg->twist.angular.x;
    b_vel_des(4) = msg->twist.angular.y;
    b_vel_des(5) = msg->twist.angular.z;
}*/

// Questo servizio permette di attivare il nodo (setta b_clik_run=true) e contestualmente lo inizializza
bool set_run_cb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
    // Sfrutto il servizio standard SetBool per ricevere la richiesta di start (true) o stop (false)

    if (!req.data) // <- se ho rishiesto stop
    {
        // semplicemente setto b_clik_run a false e ritorno.
        b_clik_run = false;
        ROS_INFO_STREAM("CLIK STOP");
        res.message = "stop";
        res.success = true;
        return true;
    }

    // Se sono qui è stato richiesto uno start
    // Devo leggere i valori correnti delle variabili di giunto per inizializzare il clik
    // in questo esempio uso ros::topic::waitForMessage per leggere un singolo messaggio dal topic /joint_states
    // ricorda che 'auto' serve per dire al compilatore di capire automaticamente il tipo (dovrebbe essre sensor_msgs::JointState::ConstPtr)
    ROS_INFO_STREAM("CLIK waiting joint state...");
    auto joint_msg = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states");

    // aggiorno la variabile globale kinematic_state con i valori letti delle variabili di giunto
    for (int i = 0; i < joint_msg->name.size(); ++i)
    {
        kinematic_state->setJointPositions(joint_msg->name[i], &joint_msg->position[i]);
    }

    // Questo blocco esegue la cinematica diretta (kinematic_state->getGlobalLinkTransform) 
    // e inizializza i valori desiderati in modo da avere errore iniziale nullo
    // Le prime due righe servono per ottenere il link rispetto al quale fare la cinematica diretta 
    // (rivedi il tutorial di MoveIt sul RobotState)
    const moveit::core::JointModelGroup *joint_model_group = kinematic_state->getRobotModel()->getJointModelGroup("yaskawa_arm");
    const moveit::core::LinkModel *last_link = kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back());
    const Eigen::Isometry3d &b_T_e = kinematic_state->getGlobalLinkTransform(last_link);
    position_des = b_T_e.translation();
    quaternion_des = b_T_e.rotation();
    oldQuaternion = quaternion_des;
    //b_vel_des.setZero();

    // Piccolo check di debug. Stampo le variabili di giunto appena salvate in kinematic_state.
    Eigen::VectorXd q;
    kinematic_state->copyJointGroupPositions(joint_model_group, q);
    ROS_INFO_STREAM("CLIK start! - q:\n"<< q);
    ROS_INFO_STREAM("position_des:\n" << position_des);
    ROS_INFO_STREAM("quaternion_des:\n" << quaternion_des.w() << " - " << quaternion_des.vec());

    for (int i = 0; i < joint_model_group->getJointModelNames().size(); i++)
        ROS_INFO_STREAM("getJointModelNames:[" << i << "]:" << joint_model_group->getJointModelNames()[i]);

    // finalmente setto b_clik_run a true e ritorno il servizio
    b_clik_run = true;
    res.success = true;
    return true;
}

int main(int argc, char *argv[])
{
    // Inizializzazione del nodo
    ros::init(argc, argv, "clik");
    ros::NodeHandle nh;

    // Guadagno e tempo di Camp.
    double Ts = 0.001;
    double clik_gain = 1.0 / Ts;

    // Costruzione delle variabili per aver modello cinematico e ultimo link del robot
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    const moveit::core::RobotModelPtr &kinematic_model = robot_model_loader.getModel();
    ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
    const moveit::core::JointModelGroup *joint_model_group = kinematic_model->getJointModelGroup("yaskawa_arm");
    kinematic_state = moveit::core::RobotStatePtr(new moveit::core::RobotState(kinematic_model));
    const moveit::core::LinkModel *last_link = kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back());
    Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0); // servirà per calcolare lo Jacobiano

    // Inizializzazione dei subscriber. N.B. non riceviamo il twist desiderato
    ros::Subscriber pose_sub = nh.subscribe("clik/desired_pose", 1, pose_des_cb);
    //ros::Subscriber twist_sub = nh.subscribe("clik/desired_twist", 1, twist_des_cb);
    // ros::Subscriber joint_sub = nh_.subscribe("joint_states", 1, joint_states_cb); PERCHE NO e si mette in attesa?

    // Inizializzo il servizio di set_run del clik 
    ros::ServiceServer srv_set_run = nh.advertiseService("clik/set_run", set_run_cb);

    // Il publisher per i comandi in giunto
    ros::Publisher cmd_pub = nh.advertise<sensor_msgs::JointState>("/cmd/joint_position", 100);

    // Il ciclo principale deve girare a periodo Ts
    ros::Rate loop_rate(1.0 / Ts);

    while(ros::ok())
    {
        loop_rate.sleep();
        // spinOnce per eseguire eventuali chiamate a servizio o callbk di messaggi in arrivo
        ros::spinOnce();

        // Se b_clik_run è falso non devo fare nulla, torno all'inizio del cilo while
        if (!b_clik_run)
        {
            continue; // va direttamente alla prossima iterazione del ciclo
        }

        // Cinematica diretta
        const Eigen::Isometry3d &b_T_e = kinematic_state->getGlobalLinkTransform(last_link);

        // Estraggo la posizione
        Eigen::Vector3d position = b_T_e.translation();

        // Estraggo il quaternione
        Eigen::Matrix3d rotation = b_T_e.rotation();
        Eigen::Quaterniond quaternion(rotation);

        // Assicuro la continuità del quaternione
        quaternion = quaternionContinuity(quaternion, oldQuaternion);
        oldQuaternion = quaternion; // <-- per il prossimo ciclo
        
        // Calcolo dell'errore in posizione e in orientamento tramite Eigen
        Eigen::Matrix<double, 6, 1> error;
        error.block<3, 1>(0, 0) = position_des - position;
        Eigen::Quaterniond deltaQ = quaternion_des * quaternion.inverse();
        error.block<3, 1>(3, 0) = deltaQ.vec();

        // Le seguenti righe implementano l'algoritmo sulla traccia, calcolando i singoli termini con Eigen
        Eigen::Matrix<double, 6, 1> vel_e = /*b_vel_des +*/ clik_gain * error;
        Eigen::MatrixXd jacobian;
        kinematic_state->getJacobian(joint_model_group, last_link, reference_point_position, jacobian);
        Eigen::VectorXd q_dot = jacobian.completeOrthogonalDecomposition().solve(vel_e);

        // Per passare da qdot a q bisogna integrare, per esempio alla Eulero
        // Copiamo i valori attuali del kinematic_state e dopo copiamo i nuovi valori di q nel kinematic_state
        Eigen::VectorXd q;
        kinematic_state->copyJointGroupPositions(joint_model_group, q);
        q = q + q_dot * Ts;
        kinematic_state->setJointGroupPositions(joint_model_group, q);

        // In questa sezione pubblichiamo il messaggio
        sensor_msgs::JointState out_msg;
        kinematic_state->copyJointGroupPositions(joint_model_group, out_msg.position);

        // infine copiamo i nomi dei giunti dal joint_model_group (PASSAGGIO FONDAMENTALE)
        out_msg.name = joint_model_group->getJointModelNames();

        // riempiamo lo stamp, utile in caso di plot
        out_msg.header.stamp = ros::Time::now();
        cmd_pub.publish(out_msg);

    }

}