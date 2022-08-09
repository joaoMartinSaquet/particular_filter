#ifndef PARTICULAR_H
#define PARTICULAR_H

#include <stdlib.h>
#include <time.h> 
#include <iostream>
#include <string>
#include <vector>
#include <random>
#include <math.h>

#include "../matplotlib-cpp/matplotlibcpp.h"
namespace plt = matplotlibcpp;
#define _USE_MATH_DEFINES


#define RANGE_X_MIN -5
#define RANGE_X_MAX 5
#define RANGE_Y_MIN 0
#define RANGE_Y_MAX 5



struct particle
{
    float x;
    float y;
    float weight;
    int cluster; 
    float min_dist; 
};

class filter
{
    public:
        filter(); //constructor 
        ~filter();//destructor
        void init();// initialisation 
        void predict(float u,float th); //predict the partcile position x+1
        void update(std::vector<particle> measure,float Rx,float Ry);
        void display();
        void showParticle(std::string title,std::string legend);
        void resample(std::string method);
        void estimate();
        float particleDistance(particle ps,particle pt);
        void kMeansClustering(int epoch,int k);
        void set_cluster_number(int k);
        float calcNeff();
        int get_size();
        void showEstimate();
        void resetWeight();
        float probMesure(float z,float mean, float var);
    private:
        std::vector<particle> particle_list;
        std::vector<float> estimatedX,estimatedY;

        int Ninit=1000;
        float xmin = 1000;
        float ymin = 1000;
        float xmax = -1000;
        float ymax = -1000;
        int clusterNumber=1;
        float sumDist;



};

#endif