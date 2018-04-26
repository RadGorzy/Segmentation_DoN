/**Na podstawie
 * @file don_segmentation.cpp
 * Difference of Normals Example for PCL Segmentation Tutorials.
 *
 * @author Yani Ioannou
 * @date 2012-09-24
 */
#include <string>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/features/don.h>
#include <pcl/filters/voxel_grid.h> //do aproksymacji

#include <pcl/common/common.h>//do getMinMax3D
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h> //
#include <pcl/io/png_io.h> // do zapisu do PNG

using namespace pcl;
using namespace std;
/*
void boundingBox (pcl::PointCloud<PointXYZRGB>& cloud, string path) //"don_cluster_8.pcd"
{
    PointXYZRGB min_pt;
    PointXYZRGB max_pt;
    pcl::getMinMax3D(cloud, min_pt, max_pt);

    int v1_1(0);
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer_1"));
    viewer->createViewPort(0.0,0.0,1.0,1.0,v1_1);//create the first view port
    viewer->setBackgroundColor (1,1,1);
    viewer->addCoordinateSystem (1.0);


    pcl::PointCloud<PointXYZRGB>::Ptr test (new pcl::PointCloud<PointXYZRGB>);
    pcl::io::loadPCDFile (path,*test);  //widac, ze uklad wspolrzednych w poszczegolnych wyodrebnonych obiektach (klastrach) jest taki sam jak w pierwotnej chmurze - duzo to ulatwia
    viewer->addPointCloud(test,"Test",0);

    viewer->addCube(min_pt.x, max_pt.x, min_pt.y,max_pt.y,min_pt.z,max_pt.z, 1.0,0.0,0.0, "cube",v1_1 );
    viewer->setRepresentationToWireframeForAllActors();
}
*/

int main ()
{
    ///The smallest scale to use in the DoN filter.
    double scale1;

    ///The largest scale to use in the DoN filter.
    double scale2;

    ///The minimum DoN magnitude to threshold by
    double threshold;

    ///segment scene into clusters with given distance tolerance using euclidean clustering
    double segradius;

    ///voxelization factor of pointcloud to use in approximation of normals
    bool approx;
    double decimation = 5; //do zmniejszania liczby punktow chmury
/*
    if (argc < 7)
    {
        cerr << "usage: " << argv[0] << "Errr, not enough arguments: inputfile smallscale largescale threshold segradius approx" << endl;
        exit (EXIT_FAILURE);
    }
*/
    /// the file to read from.
    string infile = "0000000000.pcd";
    /// small scale
    scale1 = 0.4;
    /// large scale
    scale2 = 4;      // ma najwiekszy wplyw na wydajnosc programu (poszukiwanie sasiadow w tym promieniu)
    threshold = 0.25;   // threshold for DoN magnitude - minimalna wartosc operatora delta n(P), jaka musi posiadac punkt aby nie zostal usuniety w filtracji - w artykule przyjmowano wartosc 0.25
    segradius = 0.4;   // threshold for radius segmentation -parametr do Euclidean Clustering - prog odleglosci Euclidesowej - w artykule przyjmowali ze segradius=promien maly
    approx = 1;

    // Load cloud in blob format
    pcl::PCLPointCloud2 blob;
    pcl::io::loadPCDFile (infile.c_str (), blob);
    pcl::PointCloud<PointXYZRGB>::Ptr cloud (new pcl::PointCloud<PointXYZRGB>);
    pcl::fromPCLPointCloud2 (blob, *cloud);

    //cout<<"Cloud Height:"<<cloud->height<<endl;//czyli widac, ze chmura jest unorganised

    // Create a search tree, use KDTreee for non-organized data.
    pcl::search::Search<PointXYZRGB>::Ptr tree;
    if (cloud->isOrganized ())
    {
        tree.reset (new pcl::search::OrganizedNeighbor<PointXYZRGB> ());
    }
    else
    {
        tree.reset (new pcl::search::KdTree<PointXYZRGB> (false));
    }

    // Set the input pointcloud for the search tree
    tree->setInputCloud (cloud);

    //Jezeli parapetr approx = 1 - przerzedzanie chmury
    PointCloud<PointXYZRGB>::Ptr small_cloud_downsampled;
    PointCloud<PointXYZRGB>::Ptr large_cloud_downsampled;

    if(approx){
        cout << "Downsampling point cloud for approximation" << endl;

        // Create the downsampling filtering object
        pcl::VoxelGrid<PointXYZRGB> sor;
        sor.setDownsampleAllData (false);
        sor.setInputCloud (cloud);

        // Create downsampled point cloud for DoN NN search with small scale
        small_cloud_downsampled = PointCloud<PointXYZRGB>::Ptr(new pcl::PointCloud<PointXYZRGB>);
        float smalldownsample = static_cast<float> (scale1 / decimation);
        //Eigen::Vector3f leafsize= sor.getLeafSize();
        sor.setLeafSize (smalldownsample, smalldownsample, smalldownsample);
        sor.filter (*small_cloud_downsampled);
        cout << "Using leaf size of " << smalldownsample << " for small scale, " << small_cloud_downsampled->size() << " points" << endl;

        // Create downsampled point cloud for DoN NN search with large scale
        large_cloud_downsampled = PointCloud<PointXYZRGB>::Ptr(new pcl::PointCloud<PointXYZRGB>);
        const float largedownsample = float (scale2/decimation);
        sor.setLeafSize (largedownsample, largedownsample, largedownsample);
        sor.filter (*large_cloud_downsampled);

        cout << "Using leaf size of " << largedownsample << " for large scale, " << large_cloud_downsampled->size() << " points" << endl;
    }




    if (scale1 >= scale2)
    {
        cerr << "Error: Large scale must be > small scale!" << endl;
        exit (EXIT_FAILURE);
    }

    // Compute normals using both small and large scales at each point
    pcl::NormalEstimationOMP<PointXYZRGB, PointNormal> ne; // -wykorzystuje wiele rdzeni procesora, jest jeszcze pcl::gpu::NormalEstimation - wykorzystuje GPU
    ne.setInputCloud (cloud);
    ne.setSearchMethod (tree);

    /**
     * NOTE: setting viewpoint is very important, so that we can ensure
     * normals are all pointed in the same direction!
     */
    ne.setViewPoint (std::numeric_limits<float>::max (), std::numeric_limits<float>::max (), std::numeric_limits<float>::max ());
    /**/
    if(scale1 >= scale2){
        cerr << "Error: Large scale must be > small scale!" << endl;
        exit(EXIT_FAILURE);
    }

    // calculate normals with the small scale
    cout << "Calculating normals for scale..." << scale1 << endl;
    pcl::PointCloud<PointNormal>::Ptr normals_small_scale (new pcl::PointCloud<PointNormal>);

    if(approx){
        ne.setSearchSurface(small_cloud_downsampled);
    }

    ne.setRadiusSearch (scale1);
    ne.compute (*normals_small_scale);

    // calculate normals with the large scale
    cout << "Calculating normals for scale..." << scale2 << endl;
    pcl::PointCloud<PointNormal>::Ptr normals_large_scale (new pcl::PointCloud<PointNormal>);

    if(approx){
        ne.setSearchSurface(large_cloud_downsampled);
    }

    ne.setRadiusSearch (scale2);
    ne.compute (*normals_large_scale);

    // Create output cloud for DoN results
    PointCloud<PointNormal>::Ptr doncloud (new pcl::PointCloud<PointNormal>);
    copyPointCloud<PointXYZRGB, PointNormal>(*cloud, *doncloud);

    cout << "Calculating DoN... " << endl;
    // Create DoN operator
    pcl::DifferenceOfNormalsEstimation<PointXYZRGB, PointNormal, PointNormal> don;
    don.setInputCloud (cloud);
    don.setNormalScaleLarge (normals_large_scale);
    don.setNormalScaleSmall (normals_small_scale);

    if (!don.initCompute ())
    {
        std::cerr << "Error: Could not intialize DoN feature operator" << std::endl;
        exit (EXIT_FAILURE);
    }

    // Compute DoN
    don.computeFeature (*doncloud);


    // Save DoN features
    pcl::PCDWriter writer;
    writer.write<pcl::PointNormal> ("don.pcd", *doncloud, false);
    //wizualizacja normalnych:
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewern (new pcl::visualization::PCLVisualizer ("Normals Viewer"));
    viewern->setCameraPosition	(0,0,100,0,0,0);
    //viewern->setBackgroundColor(1,1,1);
    //viewern->addPointCloud(cloud,"chmurka",0);
    //viewern->addPointCloudNormals<pcl::PointXYZRGB, pcl::PointNormal> (cloud, doncloud, 50, 2, "normals");


    // Filter by magnitude
    cout << "Filtering out DoN mag <= " << threshold << "..." << endl;

    // Build the condition for filtering
    pcl::ConditionOr<PointNormal>::Ptr range_cond (
            new pcl::ConditionOr<PointNormal> ()
    );
    range_cond->addComparison (pcl::FieldComparison<PointNormal>::ConstPtr (
            new pcl::FieldComparison<PointNormal> ("curvature", pcl::ComparisonOps::GT, threshold)) // zostawiamy tylko te punkty, ktore maja operator delta n(P) wiekszy od 0.25. - powstaja oddzielne obszary chmury
    );
    // Build the filter
    pcl::ConditionalRemoval<PointNormal> condrem (range_cond);
    condrem.setInputCloud (doncloud);

    pcl::PointCloud<PointNormal>::Ptr doncloud_filtered (new pcl::PointCloud<PointNormal>);

    // Apply filter
    condrem.filter (*doncloud_filtered);

    doncloud = doncloud_filtered;

    // Save filtered output
    std::cout << "Filtered Pointcloud: " << doncloud->points.size () << " data points." << std::endl;

    writer.write<pcl::PointNormal> ("don_filtered.pcd", *doncloud, false);
    viewern->addPointCloud<pcl::PointNormal>(doncloud,"DoN_filtered",0);
    // Filter by magnitude
    cout << "Clustering using EuclideanClusterExtraction with tolerance <= " << segradius << "..." << endl;

    pcl::search::KdTree<PointNormal>::Ptr segtree (new pcl::search::KdTree<PointNormal>);
    segtree->setInputCloud (doncloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointNormal> ec;

    ec.setClusterTolerance (segradius);
    ec.setMinClusterSize (50);
    ec.setMaxClusterSize (100000);
    ec.setSearchMethod (segtree);
    ec.setInputCloud (doncloud);
    ec.extract (cluster_indices);

    //wizualizacja i zapisywanie
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("before vs after"));

    int v0(0);

    int v1(0);
    int v2(0);

    viewer->createViewPort(0.0,0.0,0.5,1.0,v1);//create the first view port
    viewer->createViewPort(0.5,0.0,1.0,1.0,v2);//create the secondview port+
    viewer->setCameraPosition	(0,0,100,0,0,0);

    viewer->setBackgroundColor (0, 0, 0); //biale tlo, bo punkty byly czarne - wrazie braku widocznosci punktow zmienic tlo na czarne (0,0,0)
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(cloud, 0, 255, 0);
    viewer->addPointCloud (cloud, single_color, "sample cloud",v1);
    //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud",v1);
    viewer->addCoordinateSystem (1.0);

    int j = 0;
    string s;

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it, j++)
    {
        pcl::PointCloud<PointNormal>::Ptr cloud_cluster_don (new pcl::PointCloud<PointNormal>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        {
            cloud_cluster_don->points.push_back (doncloud->points[*pit]);
        }

        cloud_cluster_don->width = int (cloud_cluster_don->points.size ());
        cloud_cluster_don->height = 1;
        cloud_cluster_don->is_dense = true;


        pcl::visualization::PointCloudColorHandlerRandom<pcl::PointNormal> rgb (cloud_cluster_don); //nadawanie losowych kolorów kolejnym klastrom
        viewer->addPointCloud<pcl::PointNormal>(cloud_cluster_don, rgb, boost::lexical_cast<std::string>(j),v2);


        //Save cluster
        cout << "PointCloud representing the Cluster: " << cloud_cluster_don->points.size () << " data points." << std::endl;
        stringstream ss;
        ss << "don_cluster_" << j << ".pcd";
        writer.write<pcl::PointNormal> (ss.str (), *cloud_cluster_don, false);

        //Save PNG
        //pcl::io::savePNGFile(boost::lexical_cast<std::string>(j),*cloud_cluster_don,"normal"); //cos nie dziala - tak jakby nie posiadal zadnych fields

        //tworzenie komendy do ewentualnego wpisania w konsoli jako argument funkcji pcl_viewer (wizualizacja wynikow w jednym oknie)
        s+=string("don_cluster_")+boost::lexical_cast<std::string>(j)+".pcd"+" ";
    }


    cout<<endl<<s<<endl;

    while (!viewer->wasStopped () && !viewern->wasStopped() )
    {
        viewer->spinOnce (100);
        viewern->spinOnce(100);
        //boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }


}
