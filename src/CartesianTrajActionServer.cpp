#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "yaskawa_cross_modal/CartesianTrajAction.h"
#include "geometry_msgs/PoseStamped.h"
#include "yaskawa_cross_modal/quintic_trajectory.h"

class CartesianTrajActionServer 
{
    public:
        ros::NodeHandle nh_;
        actionlib::SimpleActionServer<yaskawa_cross_modal::CartesianTrajAction> as_;
        
        // In questo caso il publisher verrà fatto fuori dalla callback
        ros::Publisher cartesian_pub;

        //Dichiariamo costruttore/distruttore
        CartesianTrajActionServer() : as_(nh_, "CartesianTrajActionServer", boost::bind(&CartesianTrajActionServer::executeCB, this, _1), false)
        {
        }

        ~CartesianTrajActionServer() = default;

        void start()
        {
            // il publisher andrà a pubblicare su un topic intermedio letto dal nodo di inversione cinematica CLIK
            cartesian_pub = nh_.advertise<geometry_msgs::PoseStamped>("/clik/desired_pose", 1);
            as_.start();
        }

    void executeCB(const yaskawa_cross_modal::CartesianTrajGoalConstPtr &goal)
    {
        // In questo generatore non ho bisogno della posa iniziale perchè mi viene data direttamente dal goal
        /*
            Nel ciclo mi servirà il tempo dall'inizio della traiettoria t.
            Per avere t, mi salvo il tempo macchina dell'inizio della traiettoria t0.
            Poi t satà ros::Time::now()-t0
        */
        ros::Time t0 = ros::Time::now();
        ros::Duration t = ros::Time::now() - t0;

        // La nostra traiettoria sarà pubblicata con una certa frequenza.  Qui ho impostato 1000 Hz
        ros::Rate loop_rate(1000.0);

        /*
            Ciclo di "generazione traiettoria"
            cicla finchè ros è ok e non ho finito la traiettoria
            t.toSec() <= goal->duration significa t<tf.
        */
        while (ros::ok() && t.toSec() <= goal->duration)
        {
            // sleep sul rate
            loop_rate.sleep();

            // calcolo il nuovo t
            t = ros::Time::now() - t0;

            // check preemption dell'azione
            if (as_.isPreemptRequested() || !ros::ok())
            {
                ROS_INFO_STREAM("Preempted");
                as_.setPreempted();
                return;
            }

            // riempio il messaggio di comando usando la funzione quintic che ho creato 
            // nella libreria "yaskawa_cross_modal/quintic_trajectory.h"
            geometry_msgs::PoseStamped out_msg;
            out_msg.pose.position.x = qintic(t.toSec(), goal->pi.position.x, goal->pf.position.x, goal->duration);
            out_msg.pose.position.y = qintic(t.toSec(), goal->pi.position.y, goal->pf.position.y, goal->duration);
            out_msg.pose.position.z = qintic(t.toSec(), goal->pi.position.z, goal->pf.position.z, goal->duration);

            // L'orientamento è costante e lo pongo uguale a pf.
            // Nota che, a causa dei limiti di questo generatore (genera solo una traiettoria in posizione e non in orientamento)
            // l'orientamento iniziale DEVE essere lo stesso di qello finale. In questo esempio non faccio nessun controllo
            // ma sarebbe il caso di controllare che pf.orientation == pi.orientation e generare un eccezione in caso negativo
            out_msg.pose.orientation = goal->pf.orientation;

            // Sto usando la versione Stamped del messaggio posa. 
            // Sarebbe opportuno riempire l'header.stamp (il tempo attuale del messaggio)
            out_msg.header.stamp = ros::Time::now();

            // ho definito come feedback il 'tempo che manca alla fine della traiettoria'
            // ovviamente non è l'unica scelta.
            yaskawa_cross_modal::CartesianTrajFeedback feedbk_msg;
            feedbk_msg.time_left = ros::Duration(goal->duration) - t;

            // pubblico il feedback
            as_.publishFeedback(feedbk_msg);

            // pubblico il comando in cartesiano
            cartesian_pub.publish(out_msg);
        }

        // se sono arrivato fin qui, non ci sono stati errori o preeption e la traiettoria è finita
        as_.setSucceeded();
    }
};



int main(int argc, char *argv[])
{
    // Inizializzazione
    ros::init(argc, argv, "CartesianTrajActionServer");

    // Costruisco l'oggetto con il costruttore di default (nota che questo crea anche un NodeHandle)
    CartesianTrajActionServer server;

    // start dell'action server
    server.start();

    // spin per accettare le callbk dell'azione
    ros::spin();

    return 0;
}