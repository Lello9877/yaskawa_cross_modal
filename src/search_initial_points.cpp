#include <ros/ros.h>
#include <Eigen/Core>
#include <unsupported/Eigen/Splines>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

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

void sort_points(std::string path_pcl, int indice_old, int indice_new, pcl::PointCloud<pcl::PointXYZ>::Ptr sortedCloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    Eigen::Vector3d new_point, old_point, temp_point, temp_diff, differenza;
    int count, sorted_count;
    int indice_minimo;
    double valore_minimo;
    std::vector<int> indice_ordinato;
    std::vector<int> punto_candidato;
    std::vector<double> distanza_decrescente;
    std::vector<double> distance;
    std::vector<double> angle;
    bool presente, trovato, entrato;
    double distanza, angolo, prod, minima;

    if(pcl::io::loadPCDFile<pcl::PointXYZ>(path_pcl, *cloud) != 0) { return; }
    // sortedCloud->points.resize(cloud->points.size());
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
                if(distanza < 0.08)
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

                    if(angle.at(l) < 0.7)
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

int main(int argc, char **argv)
{

    ros::init(argc, argv, "search_initial_points");
    ros::NodeHandle nh;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr sortedCloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr interpolatedCloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr tempInterpolatedCloud(new pcl::PointCloud<pcl::PointXYZ>());
    int indice_old, indice_new;
    Eigen::Vector3d differenza;
    std::string path_visuale = "/home/workstation2/ws_cross_modal/bags/PCL_visuale_spirale_proc.pcd";
    std::string path_tattile = "/home/workstation2/ws_cross_modal/bags/PCL_centroide2_spirale.pcd";
    if(pcl::io::loadPCDFile<pcl::PointXYZ>(path_visuale, *cloud) != 0) { return -1; }

    // Porzione di codice per la scelta di una direzione
    int indice_iniziale = 0;
    double distanza, soglia;
    double prod;
    double angolo;
    bool trovato;
    std::vector<int> indice_vicino;
    std::vector<double> distanza_vicino;
    std::vector<Eigen::Vector3d> direzione;
    soglia = 0.08;

    for(int i = 1; i < cloud->points.size(); i++)
    {
        distanza = euclideanDistance(cloud->points.at(indice_iniziale).x, cloud->points.at(indice_iniziale).y, cloud->points.at(indice_iniziale).z, cloud->points.at(i).x, cloud->points.at(i).y, cloud->points.at(i).z);
        if(distanza < soglia)
        {
            distanza_vicino.push_back(distanza);
            indice_vicino.push_back(i);
            // std::cout << i << std::endl;
        }

    }

    for(int i = 0; i < indice_vicino.size(); i++)
    {
        differenza.x() = cloud->points.at(indice_vicino.at(i)).x - cloud->points.at(indice_iniziale).x;
        differenza.y() = cloud->points.at(indice_vicino.at(i)).y - cloud->points.at(indice_iniziale).y;
        differenza.z() = cloud->points.at(indice_vicino.at(i)).z - cloud->points.at(indice_iniziale).z;
        direzione.push_back(differenza);
    }

    if(direzione.size() > 1)
    {
        trovato = false;
        for(int i = 0; i < direzione.size()-1 && trovato == false; i++)
        {
            for(int j = i+1; j < direzione.size(); j++)
            {
                prod = direzione.at(i).x() * direzione.at(j).x() + direzione.at(i).y() * direzione.at(j).y() + direzione.at(i).z() * direzione.at(j).z();
                angolo = acos(prod/(direzione.at(i).norm()*direzione.at(j).norm()));
                if(angolo < 1  || angolo > 3.12414)
                {
                    trovato = true;
                    indice_old = indice_iniziale;
                    indice_new = indice_vicino.at(j);
                    break;
                }
            }
        }
    } 
    else if(direzione.size() == 1)
    {
        indice_old = indice_iniziale; 
        indice_new = indice_vicino.at(0); 
    }
    else { std::cout << "Nessuna direzione trovata!!!" << std::endl; return -1; }

    std::cout << "Old: " << indice_old << std::endl << "New: " << indice_new << std::endl;
    sort_points(path_visuale, indice_old, indice_new, tempCloud);
    spline(tempCloud, tempInterpolatedCloud, 400);
    pcl::io::savePCDFile("/home/workstation2/ws_cross_modal/bags/PCL_visuale_spirale_esp.pcd", *tempInterpolatedCloud);

    Eigen::Vector3d point_old, point_new, point_temp;
    point_old.x() = tempCloud->points.at(tempCloud->points.size()-1).x;
    point_old.y() = tempCloud->points.at(tempCloud->points.size()-1).y;
    point_old.z() = tempCloud->points.at(tempCloud->points.size()-1).z;
    point_new.x() = tempCloud->points.at(tempCloud->points.size()-2).x;
    point_new.y() = tempCloud->points.at(tempCloud->points.size()-2).y;
    point_new.z() = tempCloud->points.at(tempCloud->points.size()-2).z;

    for(int i = 0; i < cloud->points.size(); i++)
    {   
        point_temp.x() = cloud->points.at(i).x;
        point_temp.y() = cloud->points.at(i).y;
        point_temp.z() = cloud->points.at(i).z;
        if(point_old == point_temp) { indice_old = i; }
        if(point_new == point_temp) { indice_new = i; }
    }
    std::cout << "Old: " << indice_old << std::endl << "New: " << indice_new << std::endl;


    // indice_old = 26; indice_new = 27;
    sort_points(path_visuale, indice_old, indice_new, sortedCloud);
    spline(sortedCloud, interpolatedCloud, 400);
    pcl::io::savePCDFile("/home/workstation2/ws_cross_modal/bags/PCL_visuale_spirale_spline.pcd", *interpolatedCloud);

    return 0;
}