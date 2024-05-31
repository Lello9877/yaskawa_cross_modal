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

    if(pcl::io::loadPCDFile<pcl::PointXYZ>("/home/workstation2/ws_cross_modal/bags/PCL_visuale_retta_proc.pcd", *cloud) != 0) { return -1; }
    Eigen::Vector3d new_point, old_point, temp_point, temp_diff;
    // prova->width = 6;
    // prova->height = 1;
    // prova->points.push_back(cloud->points.at(29));
    // prova->points.push_back(cloud->points.at(24));
    // prova->points.push_back(cloud->points.at(25));
    // prova->points.push_back(cloud->points.at(27));
    // prova->points.push_back(cloud->points.at(28));
    // prova->points.push_back(cloud->points.at(26));
    // pcl::io::savePCDFile("/home/workstation2/ws_cross_modal/bags/prova.pcd", *prova);

    // for(int i = 0; i < cloud->points.size(); i++)
    //     std::cout << cloud->points.at(i) << std::endl << std::endl;

    // // Ricerca del punto iniziale che ha coordinata x maggiore (terna base)
    // for (int i = 0; i < cloud->points.size(); i++)
    // {
    //     if (new_point.x() <= cloud->points.at(i).x)
    //     {
    //         new_point.x() = cloud->points.at(i).x;
    //         new_point.y() = cloud->points.at(i).y;
    //         new_point.z() = cloud->points.at(i).z;
    //         indice_iniziale = i;
    //     }
    // }

    int count = 2;
    int indice_minimo;
    double valore_minimo;
    int indice_new, indice_old;

    // Codice per trovare i punti iniziali (Visuale: Spirale)
    // std::vector<double> coordinate;
    // for(int i = 0; i < cloud->points.size(); i++)
    //     coordinate.push_back(cloud->points.at(i).x);
    // std::pair<int,int> punto_inziale = findTwoLargestIndices(coordinate);
    // indice_old = punto_inziale.first;
    // indice_new = punto_inziale.second;

    // Spirale: new = 21, old = 22 (SOLO ANGOLO)
    // Retta: new = 1, old = 0 (SOLO DISTANZA)
    // Parabola: new = 1, old = 0 (SOLO DISTANZA)
    // Cerchio: new = 7, old = 9 (DISTANZA e ANGOLO)
    // Spirale visuale: new = 27, old = 26, distanza < 0.07
    // Cerchio visuale: new = 11, old = 13
    // Parabola visuale: new = 13, old = 14
    // Retta visuale: new = 1, old = 0
    indice_new = 1;
    indice_old = 0;
    sortedCloud->points.resize(cloud->points.size());
    sortedCloud->points.at(0) = cloud->points.at(indice_old);
    sortedCloud->points.at(1) = cloud->points.at(indice_new);
    new_point.x() = cloud->points.at(indice_new).x;
    new_point.y() = cloud->points.at(indice_new).y;
    new_point.z() = cloud->points.at(indice_new).z;
    old_point.x() = cloud->points.at(indice_old).x;
    old_point.y() = cloud->points.at(indice_old).y;
    old_point.z() = cloud->points.at(indice_old).z;
    count = 2;
    
    std::vector<int> indice_ordinato;
    std::vector<int> punto_candidato;
    std::vector<double> distanza_decrescente;
    std::vector<double> distance;
    std::vector<double> angle;
    indice_ordinato.push_back(indice_old);
    indice_ordinato.push_back(indice_new);
    
    bool presente;
    double distanza, angolo, prod, minima;
    Eigen::Vector3d differenza;

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
                distanza = calcDistance(new_point.x(), new_point.y(), new_point.z(), temp_point.x(), temp_point.y(), temp_point.z());
                if(distanza < 0.07)
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
        bool trovato = false;
        
        for(int k = 0; k < distanza_decrescente.size() && trovato == false; k++)
        {
            for(int l = 0; l < distance.size(); l++)
            {
                if(distanza_decrescente.at(k) == distance.at(l))
                {
                    if(angle.at(l) < 1)
                    {
                        std::cout << "Sto aggiungendo il punto di indice " << punto_candidato.at(l) << std::endl;
                        indice_ordinato.push_back(punto_candidato.at(l));
                        sortedCloud->points.at(count).x = cloud->points.at(punto_candidato.at(l)).x;
                        sortedCloud->points.at(count).y = cloud->points.at(punto_candidato.at(l)).y;
                        sortedCloud->points.at(count).z = cloud->points.at(punto_candidato.at(l)).z;
                        trovato = true;
                        break;
                    }
                }
            }
        }

        // int indice_minimo;
        // indice_minimo = min<double>(angle);
        // std::cout << "Angolo minimo: " << angle.at(indice_minimo) << std::endl;

        // indice_minimo = min<double>(distance);
        // std::cout << "Distanza minima: " << distance.at(indice_minimo) << std::endl;

        // std::cout << "Sto aggiungendo il punto di indice " << punto_candidato.at(indice_minimo) << std::endl;
        // indice_ordinato.push_back(punto_candidato.at(indice_minimo));
        // sortedCloud->points.at(count).x = cloud->points.at(punto_candidato.at(indice_minimo)).x;
        // sortedCloud->points.at(count).y = cloud->points.at(punto_candidato.at(indice_minimo)).y;
        // sortedCloud->points.at(count).z = cloud->points.at(punto_candidato.at(indice_minimo)).z;

        old_point = new_point;
        new_point.x() = sortedCloud->points.at(count).x;
        new_point.y() = sortedCloud->points.at(count).y;
        new_point.z() = sortedCloud->points.at(count).z;
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

        // sortedCloud->width = cloud->points.size();
        // sortedCloud->height = 1;
        // spline(sortedCloud, cloudInterpolated, 406);
        // pcl::io::savePCDFile("/home/workstation2/ws_cross_modal/bags/prova.pcd", *cloudInterpolated);
    }

    // for(int i = 0; i < sortedCloud->points.size(); i++)
    //     std::cout << sortedCloud->points.at(i) << std::endl << std::endl;

    sortedCloud->width = cloud->points.size();
    sortedCloud->height = 1;
    spline(sortedCloud, cloudInterpolated, 406);
    pcl::io::savePCDFile("/home/workstation2/ws_cross_modal/bags/PCL_visuale_retta_spline.pcd", *cloudInterpolated);

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