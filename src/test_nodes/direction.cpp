#include <ros/ros.h>
#include <TooN/TooN.h>
#include <TooN/SVD.h>
#include "sun_tactile_common/TactileStamped.h"
#include <std_msgs/Float32MultiArray.h>
#include <Eigen/Dense>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <geometry_msgs/Vector3.h>
#include <unsupported/Eigen/NumericalDiff>

#define RESET		"\033[0m"
#define RED		"\033[31m"
#define BOLDRED		"\033[1m\033[31m"
#define GREEN		"\033[32m"
#define BLUE		"\033[34m"

std::vector<float> sensor_data(12);

void getSensorData(const sun_tactile_common::TactileStamped& msg) {
    sensor_data = msg.tactile.data;
	// sensor_data[6] = sensor_data[4];
	// sensor_data[6] = sensor_data[7];
	// sensor_data[6] = sensor_data[8];
	double sum = 0;
	sum = sensor_data[4] + sensor_data[7] + sensor_data[8];
	sensor_data[6] = sum/3;
	// for(int i = 0; i < msg.tactile.data.size(); i++)
	// 	if(sensor_data[i] < 0) sensor_data[i] = 0;
	// sensor_data[5] = 5;
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

#define foreach BOOST_FOREACH
template <typename T>
void bag_write(std::string path, std::string topic, T data) {

    rosbag::Bag bag;
    bag.open(path, rosbag::bagmode::Append);
    bag.write(topic, ros::Time::now(), data);
    bag.close();

}

int main(int argc, char** argv){
	
	ros::init(argc,argv,"direction",ros::init_options::AnonymousName);
	
	/* Finding tactile sensor topic */
	// ros::master::V_TopicInfo t_info;
	// ros::master::getTopics(t_info);

	// std::string topic_name, sensor_name;
	// int count = 0, rows = 6, cols = 2;
	// while(count<t_info.size()){
	// 	if (t_info[count].name.find("TactileData") != std::string::npos && t_info[count].name.find("raw") == std::string::npos){
	// 		count++;
	// 	}
	// 	else{
	// 		t_info.erase(t_info.begin()+count);
	// 	}
	// }
	// printf("Tactile Sensors found: %lu", t_info.size());
	// if (t_info.size() == 0){
	// 	printf(BOLDRED "\nError:" RESET " no tactile sensor is publishing. Please, make sure sun_tactile_common_node is running.\n");
	// 	exit(-1);
	// }
	// else if (t_info.size() == 1){
	// 	topic_name = t_info[0].name;
	// }
	// else{
	// 	printf("\nList of available topics:\n" BLUE "[#] - /Topic Name\n" RESET);
	// 	for (int i=0 ; i<t_info.size() ; i++){
	// 		std::cout<<"["<<i<<"] - "<<t_info[i].name<<"\n";
	// 	}
	
	// 	int choice = -1;
	// 	while (choice>t_info.size()-1 || choice<0){
	// 		printf("\nPlease, select the Topic to use: ");
	// 		scanf("%d", &choice);
	// 		if (choice>t_info.size()-1 || choice<0){
	// 			printf(RED "A wrong number has been inserted.\n" RESET);
	// 		}
	// 	}
	// 	topic_name = t_info[choice].name;
	// }
	// sensor_name = topic_name.substr(topic_name.size()-4);
	
	// printf(GREEN "\nPublishing...\n" RESET);

	/* Initialization of subscriber, publishers and data */
	ros::NodeHandle n;
	ros::Rate loop_rate(5);
    int rows = 6, cols = 2;
	int count = 0, cnt = 0;
    std::string topic_name = "/tactile_voltage/rect";
	ros::Subscriber sensor_data_sub = n.subscribe(topic_name,1,getSensorData);
	ros::Publisher first_order_params_pub = n.advertise<std_msgs::Float32MultiArray>("first_order_params", 1);
	ros::Publisher second_order_params_pub = n.advertise<std_msgs::Float32MultiArray>("second_order_params", 1);
	ros::Publisher centroids_pub = n.advertise<std_msgs::Float32MultiArray>("centroids", 1);	// for graphic representation
	Eigen::MatrixXd mappa;
	mappa.resize(12,3);

	std_msgs::Float32MultiArray first_order_params_msg, second_order_params_msg, centroids_msg;
	first_order_params_msg.data.resize(2);								
	second_order_params_msg.data.resize(3);	
	centroids_msg.data.resize(6);		// info on cable "direction" and centroids position

	TooN::Matrix<6,2> delta_v; // deve diventare 6,2
	// TooN::Matrix<5,3> K = TooN::Zeros;
	TooN::Matrix<6,3> K = TooN::Zeros;
	TooN::SVD<> K_svd_first(6,2);
	TooN::SVD<> M_svd(12,3);
	// TooN::SVD<> K_svd_first(5,2);
	// TooN::SVD<> K_svd_second(5,3);
	TooN::Vector<2> h_sum;
    TooN::Vector<6> v_sum;
	TooN::Vector<2> h_vero;
	TooN::Vector<6> v_vero;
	TooN::Matrix<12,3> M = TooN::Zeros;
	// TooN::Vector<5> xi,xi_2,yi,yi_2,centroids = TooN::Zeros;
	TooN::Vector<6> xi, xi_2;
	TooN::Vector<2> yi, yi_2;
	TooN::Vector<6> centroids = TooN::Zeros;
	TooN::Vector<2> first_order_params = TooN::Zeros;
	TooN::Vector<3> second_order_params = TooN::Zeros;
	double h = 0.0, v = 0.0, angle = 0.0;
	double sum_v = 0.0;
	double num_centr = 0.0;		// numerator in centroids formula

	// Photoreflectors position
	// for (int i = 0; i < 5; i++) {
	// 	xi[i] = -7.1 + i*3.55;
	// 	xi_2[i] = pow(xi[i],2);		
	// 	yi[i] =  7.1 - i*3.55;
	// 	yi_2[i] = pow(yi[i],2);
	// }

    for(int i = 0; i < rows; i++)
    {
     	xi[i] = -7.1 + i*3.55;
		xi_2[i] = pow(xi[i],2);
    }

    for(int i = 0; i < cols; i++)
    {
     	yi[i] =  7.1 - i*3.55;
		yi_2[i] = pow(yi[i],2);
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

	// for(int i = 0; i < p_si.size(); i++)
	// 	std::cout << "Posizione " << i << ": " << std::endl << p_si.at(i) << std::endl << std::endl;
	

	// count = 0;
	// for(int i = 0; i < rows; i++)
	// {
	// 	xi[i] = p_si[count].x();
	// 	count += 2;
	// }

	// count = 0;
	// for(int i = 0; i < cols; i++)
	// {
	// 	yi[i] = p_si[count].y();
	// 	count += 1;
	// }

	// std::cout << xi << std::endl;
	// std::cout << yi << std::endl;

	/* Main loop */
	count = 0;
	while(ros::ok()){
		
		ros::spinOnce();
		
/*		sum_v = 0.0;
		for (int i = 0; i < 25; i++){
			sum_v += sensor_data[i];
		}*/
		bool enter = false;
		// for (int i = 0; i < rows; i++) {
		// 	for (int j = 0; j < cols; j++) {
		// 		delta_v(i,j) = sensor_data[j+5*i];
		// 		// if (delta_v(i,j)<0) delta_v(i,j) = 0;
		// 		if (delta_v(i,j)>0.1 && delta_v(i,j) < 3 && !enter) enter = true;
		// 	}
		// }

		cnt = 0;
		for(int i = 0; i < rows; i++)
		{
			for(int j = 0; j < cols; j++)
			{
				delta_v(i,j) = sensor_data[cnt];
				cnt++;
				if(delta_v(i,j) > 0.1 && delta_v(i,j) < 3 && !enter) enter = true;
				// std::cout << "Misura " << count << ": " << delta_v(i,j) << std::endl;
			}
		}

		/* Wire orientation estimation */
		if (enter){ //(sum_v > 0.09) {

			// Voltage data reshape
/*			for (int i = 0; i < 5; i++) {
				for (int j = 0; j < 5; j++) {
					delta_v(i,j) = sensor_data[j+5*i];
					if (delta_v(i,j)<0) delta_v(i,j) = 0;
				}
			}*/
			
			// Detection of the main direction
			h_sum = TooN::Zeros;
			v_sum = TooN::Zeros;
			h_vero = TooN::Zeros;
			v_vero = TooN::Zeros;
			
			// for(int i = 0; i < 5; i++) {
			// 	h_sum += delta_v[i];	// sum by rows
			// 	v_sum += delta_v.T()[i];	// sum by columns
			// }
			// h = h_sum[0];
			// v = v_sum[0];
			// for(int i = 1; i < 5; i++) {
			// 	if (h_sum[i]<h) h=h_sum[i];						
			// 	if (v_sum[i]<v) v=v_sum[i];		
			// }

            for(int i = 0; i < rows; i++)
                h_vero += delta_v[i];
			
			h_sum = h_vero/rows;

			// std::cout << "h_sum prima path + boost::to_string(count);
            for(int i = 0; i < cols; i++)
                v_vero += delta_v.T()[i];

			// std::cout << "v_sum prima della norm." << v_sum << std::endl; 

			v_sum = v_vero/cols;

			// std::cout << "v_sum dopo la norm." << v_sum << std::endl;

			h = h_sum[0];
			v = v_sum[0];
			// std::cout << "h: " << h << std::endl;
			// std::cout << "v: " << v << std::endl;

            for(int i = 1; i < cols; i++)
				if(h_sum[i] < h) h = h_sum[i];							

			for(int i = 1; i < rows; i++)
				if(v_sum[i] < v) v = v_sum[i];


				if(h>v) 
				{	// in this case is horizontal
					std::cout << "Orizzontale" << std::endl;
				
					// centroids_msg.data[0] = 0;
									
					// for(int i = 0; i < 5; i++) {
					// 	num_centr = 0.0;				 
					// 	for(int j = 0; j < 5; j++) {
					// 		num_centr += yi[j] * delta_v(j,i);
					// 	}					
					// 	centroids[i] = num_centr/h_sum[i];
					// }

					// for(int i = 0; i < rows; i++)
					// {
					// 	num_centr = 0.0;
					// 	for(int j = 0; j < cols; j++)
					// 	{
					// 		num_centr += yi[j] * delta_v(j,i);
					// 	}
					// 	centroids[i] = num_centr/h_vero[i];
					// }
					// std::cout << "Centroidi: " << centroids << std::endl;

					// K.T()[0] = xi_2;
					// K.T()[1] = xi;
					// K.T()[2] = TooN::Ones;
					// std::cout << K.slice(0,1,6,2) << std::endl;
					// K_svd_first.compute(K.slice(0,1,6,2));
					// first_order_params = K_svd_first.backsub(centroids);	// least squares solution
					// std::cout << "Parametri retta calcolati: " << first_order_params << std::endl;
					// K_svd_second.compute(K);
					// second_order_params = K_svd_second.backsub(centroids);			

				} 
				else 
				{	// in this case is vertical
					std::cout << "Verticale" << std::endl;
					// centroids_msg.data[0] = 1;

					// for(int i = 0; i < 5; i++) {
					// 	num_centr = 0.0;				
					// 	for(int j = 0; j < 5; j++) {
					// 		num_centr += xi[j] * delta_v(i,j);
					// 	}			count = 0;
					// 	centroids[i] = num_centr/v_sum[i];
					// }


					// for(int i = 0; i < cols; i++)
					// {
					// 	num_centr = 0.0;
					// 	for(int j = 0; j < rows; j++)
					// 	{
					// 		num_centr += xi[j] * delta_v(i,j);
					// 	}
					// 	centroids[i] = num_centr/v_vero[i];
					// }
					// std::cout << "Centroidi: " << centroids << std::endl;


					// K.T()[0] = yi_2;
					// K.T()[1] = yi;
					// K.T()[2] = TooN::Ones;
					// K_svd_first.compute(K.slice(0,1,2,2));
					// first_order_params = K_svd_first.backsub(centroids);	// least squares solution (here the equation is x = m*y+n)
					// std::cout << first_order_params << std::endl;
					// first_order_params[0] = 1/first_order_params[0];	// so here the equation is rewritten in terms of x (y = x/m-n/m)
					// first_order_params[1] = -first_order_params[0]*first_order_params[1];
					// K_svd_second.compute(K);
					// second_order_params = K_svd_second.backsub(centroids);
				}


			// centroids = TooN::Zeros;
			// for(int i = 0; i < p_si.size(); i++)
			// 	p_si.at(i).z() = sensor_data[i];

			// std::string path_bag = "/home/workstation2/ws_cross_modal/bags/mappa_tattile" + boost::to_string(count) + ".bag";
			
			// std::vector<geometry_msgs::Vector3> msg;
			// msg.resize(p_si.size());
			// for(int i = 0; i < p_si.size(); i++)
			// {
			// 	msg.at(i).x = p_si.at(i).x();
			// 	msg.at(i).y = p_si.at(i).y();
			// 	msg.at(i).z = p_si.at(i).z();
			// }

			// for(int i = 0; i < msg.size(); i++)
			// 	bag_write<geometry_msgs::Vector3>(path_bag, "/mappa_tattile", msg.at(i));
			// count++;

			// for(int i = 0; i < p_si.size(); i++)
			// {
			// 	M(i,0) = p_si.at(i).x();
			// 	M(i,1) = p_si.at(i).y();
			// 	M(i,2) = p_si.at(i).z();
			// }

			for(int i = 0; i < p_si.size(); i++)
				p_si.at(i).z() = sensor_data.at(i);

			for(int i = 0; i < p_si.size(); i++)
			{
				mappa(i,0) = p_si.at(i).x();
				mappa(i,1) = p_si.at(i).y();
				mappa(i,2) = p_si.at(i).z();
			}

			// std::cout << mappa.col(0).size() << std::endl;

			// double media = 0;
			// for(int i = 0; i < mappa.row(0).size(); i++)
			// {
			// 	media = mappa.col(i).mean();
			// 	std::cout << "Media colonna " << i << ": " << media << std::endl;
			// 	for(int j = 0; j < mappa.col(0).size(); j++)
			// 		mappa(j,i) = mappa(j,i) - media;
			// }

			// std::cout << "Marice Normalizzata: " << std::endl << mappa << std::endl;

			// Eigen::JacobiSVD<Eigen::MatrixXd> svd(mappa, Eigen::ComputeThinU | Eigen::ComputeThinV);
			// Eigen::MatrixXd U = svd.matrixU();
			// Eigen::MatrixXd V = svd.matrixV();
			// Eigen::VectorXd S = svd.singularValues();
			// std::cout << "Valori Singolari: " << S << std::endl;

			// Eigen::MatrixXd dV;
			// dV.resize(rows,cols);
			// int count = 0;
			// for(int i = 0; i < rows; i++)
			// 	for(int j = 0; j < cols; j++)
			// 	{
			// 		dV(i,j) = sensor_data[count];
			// 		count += 1;
			// 	}

			// std::cout << mappa << std::endl << std::endl;
			// std::cout << dV << std::endl << std::endl;
			// double x2, x1, y2, y1;

			// for(int i = 0; i < rows; i++)
			// {
			// 	for(int j = 0; j < cols; j++)
			// 	{
			// 		x1 = dV(i,j);
			// 		y1 = dV(i,j);
			// 		if(i < rows - 1)
			// 		{
			// 			x2 = dV(i+1,j);
			// 		}
			// 		else
			// 		{
			// 			x2 = dV(i-1,j);
			// 		}
			// 		if(j < cols - 1)
			// 		{
			// 			y2 = dV(i,j+1);
			// 		}
			// 		else
			// 		{
			// 			y2 = dV(i,j-1);
			// 		}

			// 	}
			// }

			// std::cout << M << std::endl;
			// M_svd.compute(M);
			// TooN::Vector<> val_sing = M_svd.get_diagonal();
			// std::cout << val_sing << std::endl << std::endl;
			// M = TooN::Zeros;

			enter = false;
			if(!askContinue("Prossima Misura?")) break;
		}
		// else{
		// 	first_order_params = TooN::makeVector(0.0,0.0);
		// 	second_order_params = TooN::makeVector(0.0,0.0,0.0);
		// }
		
		// first_order_params_msg.data[0] = first_order_params[0];
		// first_order_params_msg.data[1] = first_order_params[1];
		// second_order_params_msg.data[0] = second_order_params[0];
		// second_order_params_msg.data[1] = second_order_params[1];
		// second_order_params_msg.data[2] = second_order_params[2];
		
		// for (int i=0;i<5;i++){
		// 	centroids_msg.data[1+i]=centroids[i];
		// }
		
		// // Publish on topics
		// first_order_params_pub.publish(first_order_params_msg);
		// second_order_params_pub.publish(second_order_params_msg);
		// centroids_pub.publish(centroids_msg);

		loop_rate.sleep();
	}
	
	return 0;
}