#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "yaskawa_cross_modal/JointTrajAction.h"
#include "sensor_msgs/JointState.h"
#include "yaskawa_cross_modal/quintic_trajectory.h"


// Variabili globali che servono per capire quando è arrivato un messaggio dal topic
// joint_state, sfruttando la variabile booleana e la funzione apposita di seguito
sensor_msgs::JointState joint_state;
bool joint_state_arrived = false;

void joint_states_cb(const sensor_msgs::JointState::ConstPtr msg)
{
    joint_state = *msg;
    joint_state_arrived = true;
}

// Ogni volta che si crea un'action server bisogna creare una classe
// che ne descrive il funzionamento (con la callback nella classe)
class JointTrajActionServer
{
public:

    // Va sempre fatto il NodeHanlde prima del simple action server!
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<yaskawa_cross_modal::JointTrajAction> as_;

    // Elenco dei giunti, supponendo che l'ordine sia già corretto
    std::vector<std::string> joint_names = {"joint_s",
                                            "joint_l",
                                            "joint_e",
                                            "joint_u",
                                            "joint_r",
                                            "joint_b",
                                            "joint_t"};

    // Dichiariamo il costruttore (distruttore)
    JointTrajActionServer() : as_(nh_, "JointTrajActionServer", boost::bind(&JointTrajActionServer::executeCB, this, _1), false)
    {
    }

    ~JointTrajActionServer() = default;

    // Metodo per avviare l'action server
    void start() 
    { 
        as_.start(); 
    }

    void executeCB(const yaskawa_cross_modal::JointTrajGoalConstPtr &goal)
    {
        // Realizziamo la pubblicazione e la sottoscrizione all'interno della callback (tutto in questo nodo)
        // Pubblica comandi di posizione sul topic
        ros::Publisher joint_cmd_pub = nh_.advertise<sensor_msgs::JointState>("/cmd/joint_position", 1);

        // Qui inizia la sottosrizione (parte di lettura dal topic)
        ros::Subscriber joint_sub = nh_.subscribe("joint_states", 1, joint_states_cb);

        // Settiamo la booleana a false per essere sicuri di non prendere un messaggio vecchio
        // Il ciclo di attesa aspetta che il nuovo messaggio arrivi
        joint_state_arrived = false;
        while (ros::ok() && !joint_state_arrived) // Finchè ros è ok e non è arrivato nessun messaggio
        {
            // controllo la preemption (tipico delle action)
            if (as_.isPreemptRequested() || !ros::ok())
            {
                ROS_INFO_STREAM("Preempted");
                as_.setPreempted();
                return; 
                // return termina la callbk dell'action (perché se sto qui è arrivata una preemption)
            }
            // nel ciclo faccio lo spinOnce che mi chiama la callback del subscriber (se il messaggio è arrivato)
            ros::spinOnce();
        }

        // Salviamo la configurazione iniziale che non cambierà successivamente
        sensor_msgs::JointState qi = joint_state;
        ROS_INFO_STREAM("\n qi is:\n" << qi);

        //  Quando genereremo la traiettoria servirà il tempo attuale
        ros::Time t0 = ros::Time::now();
        ros::Duration t = ros::Time::now() - t0;

        // La nostra traiettoria sarà pubblicata con una certa frequenza. Qui ho impostato 100 Hz
        ros::Rate loop_rate(100.0);

        // Ciclo di generazione della traiettoria, finché ros è ok e non ha finito la generazione
        while (ros::ok() && t.toSec() <= goal->duration)
        {
            loop_rate.sleep();

            // calcolo il nuovo t
            t = ros::Time::now() - t0;

            if (as_.isPreemptRequested() || !ros::ok())
            {
                ROS_INFO_STREAM("Preempted");
                as_.setPreempted();
                return;
            }

            // Invochiamo la generazione della traiettoria sul giunto i-esimo
            // e riempiamo il messaggio del comando cmd opportunamente, infine pubblica il messaggio sul topic
            sensor_msgs::JointState cmd;
            cmd.position.resize(7);

            for (int i = 0; i < 7; i++)
                cmd.position[i] = qintic(t.toSec(), qi.position[i], goal->qf[i], goal->duration);

            cmd.name = joint_names;
            cmd.header.stamp = ros::Time::now();
            yaskawa_cross_modal::JointTrajFeedback feedbk_msg;
            feedbk_msg.time_left = ros::Duration(goal->duration) - t;
            as_.publishFeedback(feedbk_msg);
            joint_cmd_pub.publish(cmd);
        }

        as_.setSucceeded();
    }
};

int main(int argc, char *argv[])
{
    // init
    ros::init(argc, argv, "JointTrajActionServer");

    // costruisco l'oggetto con il costruttore di default:
    // (nota che questo crea anche un NodeHandle)
    JointTrajActionServer server;

    // start dell'action server
    server.start();

    // spin per accettare le callback dell'azione
    ros::spin();

    return 0;
}