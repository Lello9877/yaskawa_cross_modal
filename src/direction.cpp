#include <ros/ros.h>
#include <TooN/TooN.h>
#include <TooN/SVD.h>
#include "sun_tactile_common/TactileStamped.h"
#include <std_msgs/Float32MultiArray.h>
#include <Eigen/Dense>

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
	for(int i = 0; i < msg.tactile.data.size(); i++)
		if(sensor_data[i] < 0) sensor_data[i] = 0;
	// sensor_data[5] = 5;
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
	ros::Rate loop_rate(20);
    int rows = 6, cols = 2;
	int count = 0;
    std::string topic_name = "/tactile_voltage/rect";
	ros::Subscriber sensor_data_sub = n.subscribe(topic_name,1,getSensorData);
	ros::Publisher first_order_params_pub = n.advertise<std_msgs::Float32MultiArray>("first_order_params", 1);
	ros::Publisher second_order_params_pub = n.advertise<std_msgs::Float32MultiArray>("second_order_params", 1);
	ros::Publisher centroids_pub = n.advertise<std_msgs::Float32MultiArray>("centroids", 1);	// for graphic representation

	std_msgs::Float32MultiArray first_order_params_msg, second_order_params_msg, centroids_msg;
	first_order_params_msg.data.resize(2);								
	second_order_params_msg.data.resize(3);	
	centroids_msg.data.resize(6);		// info on cable "direction" and centroids position

	TooN::Matrix<6,2> delta_v; // deve diventare 6,2
	// TooN::Matrix<5,3> K = TooN::Zeros;
	TooN::Matrix<6,3> K = TooN::Zeros;
	TooN::SVD<> K_svd_first(6,2);
	// TooN::SVD<> K_svd_first(5,2);
	// TooN::SVD<> K_svd_second(5,3);
	TooN::Vector<2> h_sum;
    TooN::Vector<6> v_sum;
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

	// for(int i = 0; i < rows; i++)
    // {
    //  xi[i] = -8.875 + i*3.55;
	// 	xi_2[i] = pow(xi[i],2);
	// 	yi[i] =  8.875 - i*3.55;
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

    // // Generazione delle coordinate (xi,yi) in terna reference_taxel
    // std::vector<Eigen::Vector3d> p_si;
    // {
    //     int count = 0;
    //     double refx = 0;
    //     double refy = 0;
    //     double offset = 0.0035;

    //     for(int j = 0; j < rows; j++) {
    //         refy = 0;
    //         for(int k = 0; k < cols; k++) {
    //             Eigen::Vector3d temp(refx,refy,0);
    //             p_si.push_back(temp);
    //             refy = refy + offset;
    //             count++;
    //         }
    //         refx = refx + offset;
    //     }  
    // }

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

	std::cout << xi << std::endl;
	std::cout << yi << std::endl;

	/* Main loop */
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

		count = 0;
		for(int i = 0; i < rows; i++)
		{
			for(int j = 0; j < cols; j++)
			{
				delta_v(i,j) = sensor_data[count];
				count++;
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
                h_sum += delta_v[i];

			// std::cout << "h_sum prima della norm." << h_sum << std::endl; 

			h_sum = h_sum/rows;

			// std::cout << "h_sum dopo la norm." << h_sum << std::endl;

            for(int i = 0; i < cols; i++)
                v_sum += delta_v.T()[i];

			// std::cout << "v_sum prima della norm." << v_sum << std::endl; 

			v_sum = v_sum/cols;

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
					// 		num_centr += yi[i] * delta_v(j,i);
					// 	}
					// 	centroids[i] = num_centr/h_sum[i];
					// }
					// std::cout << "Centroidi: " << centroids << std::endl;

					// for(int i = 0; i < rows; i++)
					// {
					// 	num_centr = 0.0;
					// 	for(int j = 0; j < cols; j++)
					// 	{
					// 		num_centr += xi[i] * delta_v(j,i);
					// 	}
					// 	centroids[i] = num_centr/v_sum[i];
					// }

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
					// 	}
					// 	centroids[i] = num_centr/v_sum[i];
					// }
					// K.T()[0] = yi_2;
					// K.T()[1] = yi;
					// K.T()[2] = TooN::Ones;
					// K_svd_first.compute(K.slice(0,1,5,2));
					// first_order_params = K_svd_first.backsub(centroids);	// least squares solution (here the equation is x = m*y+n)
					// first_order_params[0] = 1/first_order_params[0];	// so here the equation is rewritten in terms of x ( y = x/m-n/m)
					// first_order_params[1] = -first_order_params[0]*first_order_params[1];
					// K_svd_second.compute(K);
					// second_order_params = K_svd_second.backsub(centroids);
				}
			enter = false;
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