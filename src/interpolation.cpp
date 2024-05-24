#include <ros/ros.h>
#include <Eigen/Core>
#include <unsupported/Eigen/Splines>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

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

template <typename T>
int min(std::vector<T> vec) 
{
    T min = vec.at(0);
    for(int i = 0; i < vec.size(); i++)
        if(vec.at(i) <= min)
            min = i;
    return min;
}

double calcDistance(double x1, double y1, double z1, double x2, double y2, double z2)
{
    double x = x1 - x2; // calculating number to square in next step
    double y = y1 - y2;
    double z = z1 - z2;
    double dist;

    dist = pow(x,2) + pow(y,2) + pow(z,2); // calculating Euclidean distance
    dist = sqrt(dist);

    return dist;
}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "interpolation");
    ros::NodeHandle nh;

    // Prelevo i punti di via dalla point cloud tattile
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInterpolated(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr sortedCloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr prova(new pcl::PointCloud<pcl::PointXYZ>());

    // Indici Ordinati Spirale: 6, 11, 14, 18, 19, 17, 15, 12, 9, 7, 4, 2, 1, 0, 3, 5, 8, 10, 13, 16, 20, 21, 22
    // Indici Ordinati Cerchio: 13, 14, 15, 18, 17, 16, 12, 11, 10, 8, 6, 4, 2, 1, 0, 3, 5, 7, 9
    // Indici Ordinati Parabola: 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 12, 11
    // if(pcl::io::loadPCDFile<pcl::PointXYZ>("/home/workstation2/ws_cross_modal/bags/PCL_centroide2_spirale.pcd", *cloud) != 0) { return -1; }
    // int indice_spirale[cloud->points.size()] = {6, 11, 14, 18, 19, 17, 15, 12, 9, 7, 4, 2, 1, 0, 3, 5, 8, 10, 13, 16, 20, 21, 22};
    // *sortedCloud = *cloud;
    // for(int i = 0; i < cloud->points.size(); i++)
    // {
    //     sortedCloud->points.at(i).x = cloud->points.at(indice_spirale[i]).x;
    //     sortedCloud->points.at(i).y = cloud->points.at(indice_spirale[i]).y;
    //     sortedCloud->points.at(i).z = cloud->points.at(indice_spirale[i]).z;
    // }

    // int num_points = 600;
    // spline(sortedCloud, cloudInterpolated, num_points);
    // pcl::io::savePCDFile("/home/workstation2/ws_cross_modal/bags/PCL_centroide2_spirale_spline.pcd", *cloudInterpolated);

    // if(pcl::io::loadPCDFile<pcl::PointXYZ>("/home/workstation2/ws_cross_modal/bags/PCL_centroide2_cerchio.pcd", *cloud) != 0) { return -1; }
    // int indice_cerchio[cloud->points.size()] = {13, 14, 15, 18, 17, 16, 12, 11, 10, 8, 6, 4, 2, 1, 0, 3, 5, 7, 9};
    // *sortedCloud = *cloud;
    // for(int i = 0; i < cloud->points.size(); i++)
    // {
    //     sortedCloud->points.at(i).x = cloud->points.at(indice_cerchio[i]).x;
    //     sortedCloud->points.at(i).y = cloud->points.at(indice_cerchio[i]).y;
    //     sortedCloud->points.at(i).z = cloud->points.at(indice_cerchio[i]).z;
    // }

    // spline(sortedCloud, cloudInterpolated, num_points);
    // pcl::io::savePCDFile("/home/workstation2/ws_cross_modal/bags/PCL_centroide2_cerchio_spline.pcd", *cloudInterpolated);

    // if(pcl::io::loadPCDFile<pcl::PointXYZ>("/home/workstation2/ws_cross_modal/bags/PCL_centroide2_parabola.pcd", *cloud) != 0) { return -1; }
    // int indice_parabola[cloud->points.size()] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 11, 10};
    // *sortedCloud = *cloud;
    // for(int i = 0; i < cloud->points.size(); i++)
    // {
    //     sortedCloud->points.at(i).x = cloud->points.at(indice_parabola[i]).x;
    //     sortedCloud->points.at(i).y = cloud->points.at(indice_parabola[i]).y;
    //     sortedCloud->points.at(i).z = cloud->points.at(indice_parabola[i]).z;
    // }

    // spline(sortedCloud, cloudInterpolated, num_points);
    // pcl::io::savePCDFile("/home/workstation2/ws_cross_modal/bags/PCL_centroide2_parabola_spline.pcd", *cloudInterpolated);

    // if(pcl::io::loadPCDFile<pcl::PointXYZ>("/home/workstation2/ws_cross_modal/bags/PCL_centroide2_retta.pcd", *cloud) != 0) { return -1; }
    // spline(cloud, cloudInterpolated, num_points);
    // pcl::io::savePCDFile("/home/workstation2/ws_cross_modal/bags/PCL_centroide2_retta_spline.pcd", *cloudInterpolated);

    if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/workstation2/ws_cross_modal/bags/PCL_centroide2_spirale.pcd", *cloud) != 0)
    {
        return -1;
    }
    Eigen::Vector3d starting_point, prev_point, succ_point, temp_point, temp_diff;
    starting_point.x() = cloud->points.at(0).x;
    starting_point.y() = cloud->points.at(0).y;
    starting_point.z() = cloud->points.at(0).z;
    int indice_iniziale = 0;

    // Ricerca del punto iniziale che ha coordinata x maggiore (terna base)
    for (int i = 0; i < cloud->points.size(); i++)
    {
        if (starting_point.x() <= cloud->points.at(i).x)
        {
            starting_point.x() = cloud->points.at(i).x;
            starting_point.y() = cloud->points.at(i).y;
            starting_point.z() = cloud->points.at(i).z;
            indice_iniziale = i;
        }
    }

    int count = 0;
    sortedCloud->points.resize(cloud->points.size());
    sortedCloud->points.at(0) = cloud->points.at(22);
    starting_point.x() = cloud->points.at(22).x;
    starting_point.y() = cloud->points.at(22).y;
    starting_point.z() = cloud->points.at(22).z;
    prev_point = starting_point;
    std::vector<int> indice;
    std::vector<int> punto_candidato;
    std::vector<double> distance;
    std::vector<double> angle;
    // punto_candidato.resize(cloud->points.size());
    // indice.resize(cloud->points.size());
    indice.push_back(indice_iniziale);
    std::cout << indice.size() << std::endl;
    // distance.resize(cloud->points.size());
    
    bool presente;
    double distanza, angolo, prod, minima;
    Eigen::Vector3f differenza;

    while(count < cloud->points.size())
    {
        for (int i = 0; i < cloud->points.size(); i++)
        {
            // std::cout << "ciao indice i: " << i << std::endl;
            presente = false;
            for(int j = 0; j < indice.size(); j++) 
            {
                if(i == indice.at(j)) { presente = true; /*std::cout << "sono un indice presente: " << i << std::endl;*/ }
                // std::cout << "dimensione del vettore degli indice: " << indice.size() << std::endl;
                // std::cout << indice.at(j) << std::endl;
            }

            if(!presente)
            {
                temp_point.x() = cloud->points.at(i).x;
                temp_point.y() = cloud->points.at(i).y;
                temp_point.z() = cloud->points.at(i).z;
                distanza = calcDistance(prev_point.x(), prev_point.y(), prev_point.z(), temp_point.x(), temp_point.y(), temp_point.z());
                if(distanza < 0.7)
                {
                    punto_candidato.push_back(i);
                    distance.push_back(distanza);
                    temp_diff = temp_point - prev_point;
                    prod = prev_point.x() * temp_diff.x() + prev_point.y() * temp_diff.y() + prev_point.z() * temp_diff.z();
                    angolo = acos(prod/(prev_point.norm() * temp_diff.norm()));
                    angle.push_back(angolo);
                }
            }
        }

        minima = min<double>(distance);
        //punto_candidato.resize(distance.size());
        std::cout << punto_candidato.size() << std::endl;
        std::cout <<  distance.size() << std::endl;
        std::cout << angle.size() << std::endl;
        for(int k = 0; k < punto_candidato.size(); k++)
        {
            std::cout << "ciao k: " << k << std::endl;
            if(angle.at(k) < 0.15)
            {
                std::cout << "Sto aggiungendo il punto di indice " << punto_candidato.at(k) << std::endl;
                indice.push_back(punto_candidato.at(k));
                sortedCloud->points.at(count).x = cloud->points.at(punto_candidato.at(k)).x;
                sortedCloud->points.at(count).y = cloud->points.at(punto_candidato.at(k)).y;
                sortedCloud->points.at(count).z = cloud->points.at(punto_candidato.at(k)).z;
                break;
            }
        }
        // int indice_angolo_minimo = min<double>(angle);
        // std::cout << "Sto aggiungendo l'elemento di indice " << punto_candidato.at(indice_angolo_minimo) << std::endl;
        // indice.push_back(punto_candidato.at(indice_angolo_minimo));
        // sortedCloud->points.at(count).x = cloud->points.at(punto_candidato.at(indice_angolo_minimo)).x;
        // sortedCloud->points.at(count).y = cloud->points.at(punto_candidato.at(indice_angolo_minimo)).y;
        // sortedCloud->points.at(count).z = cloud->points.at(punto_candidato.at(indice_angolo_minimo)).z;
        std::cout << "contatore: " << count << std::endl;
        prev_point.x() = sortedCloud->points.at(count).x;
        prev_point.y() = sortedCloud->points.at(count).y;
        prev_point.z() = sortedCloud->points.at(count).z;
        distance.clear();
        angle.clear();
        punto_candidato.clear();
        distance.resize(0);
        angle.resize(0);
        punto_candidato.resize(0);
        count++;
    }

    sortedCloud->width = cloud->points.size();
    sortedCloud->height = 1;
    spline(sortedCloud, cloudInterpolated, 400);
    pcl::io::savePCDFile("/home/workstation2/ws_cross_modal/bags/PCL_centroide2_spirale_spline.pcd", *cloudInterpolated);

    // for(int i = 0; i < cloud->points.size(); i++) {
    //     temp_point.x() = cloud->points.at(i).x;
    //     temp_point.y() = cloud->points.at(i).y;
    //     temp_point.z() = cloud->points.at(i).z;
    //     distanza = calcDistance(prev_point.x(), prev_point.y(), prev_point.z(), temp_point.x(), temp_point.y(), temp_point.z());
    //     //std::cout << "Distanza tra il punto 22 con il punto di indice " << i << ": " << distanza << std::endl;
    //     prod = prev_point.x() * temp_point.x() + prev_point.y() * temp_point.y() + prev_point.z() * temp_point.z();
    //     angolo = acos(prod/(prev_point.norm() * temp_point.norm()));
    //     std::cout << "Angolo tra il punto 22 e il punto di indice " << i << ": " << angolo << std::endl;
    // }

    return 0;
}