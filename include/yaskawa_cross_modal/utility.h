#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <Eigen/Dense>
#include <unsupported/Eigen/Polynomials>
#include <unsupported/Eigen/Splines>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/region_growing_rgb.h>
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

void segmentazione(std::string path, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb)
{
    pcl::io::loadPCDFile(path, *cloud_rgb);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    kdtree->setInputCloud(cloud_rgb);
    pcl::RegionGrowingRGB<pcl::PointXYZRGB> clustering;
	clustering.setInputCloud(cloud_rgb);
	clustering.setSearchMethod(kdtree);
    clustering.setMinClusterSize(600);
    clustering.setDistanceThreshold(10);
    clustering.setPointColorThreshold(6);
    clustering.setRegionColorThreshold(5);
    std::vector <pcl::PointIndices> clusters;
	clustering.extract(clusters);

    // For every cluster...
	int currentClusterNum = 1;
	for (std::vector<pcl::PointIndices>::const_iterator i = clusters.begin(); i != clusters.end(); ++i)
	{
		// ...add all its points to a new cloud...
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
		for (std::vector<int>::const_iterator point = i->indices.begin(); point != i->indices.end(); point++)
			cluster->points.push_back(cloud_rgb->points[*point]);
		cluster->width = cluster->points.size();
		cluster->height = 1;
		cluster->is_dense = true;

		// ...and save it to disk.
		if (cluster->points.size() <= 0)
			break;
		std::cout << "Cluster " << currentClusterNum << " has " << cluster->points.size() << " points." << std::endl;
		std::string fileName = "/home/workstation2/ws_cross_modal/cluster" + boost::to_string(currentClusterNum) + ".pcd";
		pcl::io::savePCDFile(fileName, *cluster);

		currentClusterNum++;
	}

    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud = clustering.getColoredCloud ();
    // pcl::visualization::CloudViewer viewer ("Cluster viewer");
    // viewer.showCloud(colored_cloud);

    // while (!viewer.wasStopped ())
    //     std::this_thread::sleep_for(100us);

}

template <typename T>
int min(std::vector<T> vec) 
{
    int indice_minimo = 0;
    T valore_minimo = vec.at(0);
    for(int l = 0; l < vec.size(); l++)
    {
        if(vec.at(l) <= valore_minimo)
        {
            valore_minimo = vec.at(l);
            indice_minimo = l;
        }
    }
    return indice_minimo;
}

double euclideanDistance(double x1, double y1, double z1, double x2, double y2, double z2)
{
    double x = x1 - x2;
    double y = y1 - y2;
    double z = z1 - z2;
    double dist;

    dist = pow(x,2) + pow(y,2) + pow(z,2);
    dist = sqrt(dist);

    return dist;
}

typedef Eigen::Spline<float, 3> Spline3d;
void spline(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInterpolated, int num_points)
{
    std::vector<Eigen::Vector3d> waypoints;
    for (int i = 0; i < cloud->points.size(); i++)
    {
        Eigen::Vector3d punto(cloud->points.at(i).x, cloud->points.at(i).y, cloud->points.at(i).z);
        waypoints.push_back(punto);
    }

    Eigen::MatrixXf points(3, waypoints.size());
    int row_index = 0;
    for (auto const way_point : waypoints)
    {
        points.col(row_index) << way_point[0], way_point[1], way_point[2];
        row_index++;
    }

    Spline3d spline = Eigen::SplineFitting<Spline3d>::Interpolate(points, 3);
    float time_ = 0;
    *cloudInterpolated = *cloud;
    cloudInterpolated->points.resize(cloud->points.size() + num_points);
    cloudInterpolated->width = cloud->points.size() + num_points;

    for (int i = cloud->points.size(); i < cloud->points.size() + num_points; i++)
    {
        time_ += 1.0 / (num_points * 1.0);
        Eigen::VectorXf values = spline(time_);
        // std::cout << values << std::endl << std::endl;
        cloudInterpolated->points.at(i).x = values.x();
        cloudInterpolated->points.at(i).y = values.y();
        cloudInterpolated->points.at(i).z = values.z();
    }
}

void sort_points(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int indice_old, int indice_new, pcl::PointCloud<pcl::PointXYZ>::Ptr sortedCloud)
{
    Eigen::Vector3d new_point, old_point, temp_point, temp_diff, differenza;
    int count, sorted_count;
    // int indice_minimo;
    // double valore_minimo;
    std::vector<int> indice_ordinato;
    std::vector<int> punto_candidato;
    std::vector<double> distanza_decrescente;
    std::vector<double> distance;
    std::vector<double> angle;
    bool presente, trovato, entrato;
    double distanza, angolo, prod, minima;

    indice_ordinato.push_back(indice_old);
    indice_ordinato.push_back(indice_new);

    sortedCloud->points.push_back(cloud->points.at(indice_old));
    sortedCloud->points.push_back(cloud->points.at(indice_new));
    old_point.x() = cloud->points.at(indice_old).x;
    old_point.y() = cloud->points.at(indice_old).y;
    old_point.z() = cloud->points.at(indice_old).z;
    new_point.x() = cloud->points.at(indice_new).x;
    new_point.y() = cloud->points.at(indice_new).y;
    new_point.z() = cloud->points.at(indice_new).z;
    count = 2;
    sorted_count = count;;

    while(count < cloud->points.size())
    {
        differenza = new_point - old_point;
        for(int i = 0; i < cloud->points.size(); i++)
        {
            // std::cout << "ciao indice i: " << i << std::endl;
            presente = false;
            for(int j = 0; j < indice_ordinato.size(); j++) 
            {
                if(i == indice_ordinato.at(j)) { presente = true; /*std::cout << "sono un indice presente: " << i << std::endl;*/ break; }
                // std::cout << "dimensione del vettore degli indice: " << indice.size() << std::endl;
                // std::cout << indice.at(j) << std::endl;
            }

            if(!presente)
            {
                temp_point.x() = cloud->points.at(i).x;
                temp_point.y() = cloud->points.at(i).y;
                temp_point.z() = cloud->points.at(i).z;
                distanza = euclideanDistance(new_point.x(), new_point.y(), new_point.z(), temp_point.x(), temp_point.y(), temp_point.z());
                if(distanza < 0.06)
                {
                    std::cout << "La distanza candidata " << distanza << " è associata al punto di indice " << i << std::endl;
                    punto_candidato.push_back(i);
                    distance.push_back(distanza);
                    temp_diff = temp_point - new_point;
                    prod = differenza.x() * temp_diff.x() + differenza.y() * temp_diff.y() + differenza.z() * temp_diff.z();
                    angolo = acos(prod/(differenza.norm() * temp_diff.norm()));
                    angle.push_back(angolo);
                    std::cout << "Angolo associato al punto di indice " << i << ": " << angolo << std::endl;
                }
            }
        }

        distanza_decrescente = distance;
        std::sort(distanza_decrescente.begin(), distanza_decrescente.end());
        trovato = false;
        
        for(int k = 0; k < distanza_decrescente.size() && trovato == false; k++)
        {
            for(int l = 0; l < distance.size(); l++)
            {
                if(distanza_decrescente.at(k) == distance.at(l))
                {   // 1 grado = 0.01745 rad
                    // 0.8, giocando con quella precedente: 1.2
                    if(angle.at(l) < 1)
                    {
                        std::cout << "Sto aggiungendo il punto di indice " << punto_candidato.at(l) << std::endl;
                        indice_ordinato.push_back(punto_candidato.at(l));
                        sortedCloud->points.push_back(cloud->points.at(punto_candidato.at(l)));
                        trovato = true;
                        sorted_count++;
                        break;
                    }
                }
            }
        }

        if(trovato == true)
        {
            old_point = new_point;
            new_point.x() = sortedCloud->points.at(sorted_count-1).x;
            new_point.y() = sortedCloud->points.at(sorted_count-1).y;
            new_point.z() = sortedCloud->points.at(sorted_count-1).z;
        }

        std::cout << "contatore: " << count << std::endl << std::endl;
        distance.clear();
        angle.clear();
        punto_candidato.clear();
        distanza_decrescente.clear();
        distanza_decrescente.resize(0);
        distance.resize(0);
        angle.resize(0);
        punto_candidato.resize(0);
        count++;

    }
    sortedCloud->width = sortedCloud->points.size();
    sortedCloud->height = 1;
}