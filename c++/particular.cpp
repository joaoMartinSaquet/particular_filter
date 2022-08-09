#include "particular.h"

// to compile the code use g++ particular.cpp -I/usr/include/python3.8 -lpython3.8 and then ./file.out

void sleepcp(int milliseconds) // Cross-platform sleep function
{
    clock_t time_end;
    time_end = clock() + milliseconds * CLOCKS_PER_SEC/1000;
    while (clock() < time_end)
    {
    }
}

filter::filter()
{

}

filter::~filter()
{

}

void filter::showParticle(std::string title,std::string legend)
{
    std::vector<float> x,y;
    for(auto p : this->particle_list)
    {
        x.push_back(p.x);
        y.push_back(p.y);
    }
    //plt::xlim(this->xmin-1,this->xmax+1);
    //plt::ylim(this->ymin-1,this->ymax+1);
    plt::scatter(x,y);
    //plt::ylabel(legend);
    //plt::legend();


}

void filter::init()
{
    std::default_random_engine generator;
    std::normal_distribution<float> x_randn(0,5); 
    std::normal_distribution<float> y_randn(3,5); //change mean 
    for(int i=0;i<this->Ninit;i++)
    {
        particle tmp;
        tmp.x = x_randn(generator);
        tmp.y = y_randn(generator);
        tmp.weight = 1.0/this->Ninit;
        tmp.cluster = 0;
        tmp.min_dist = std::numeric_limits<float>::max();
        this->particle_list.push_back(tmp);
        if(tmp.x >= this->xmax)
        {
            this->xmax = tmp.x;
        }

        if(tmp.y >= this->ymax)
        {
            this->ymax = tmp.y;
        }

        if(tmp.x <= this->xmin)
        {
            this->xmin = tmp.x;
        }

        if(tmp.y <= this->ymin)
        {
            ymin = tmp.y;
        }

    }
} 

void filter::predict(float u,float th)
{
    // u is for the distance desired, and theta for the angular command
    // hypth only the robot command is noisy so there is same noise for all particle 

    std::normal_distribution<float> dx_randn(0,0.1);
    std::normal_distribution<float> dy_randn(0,0.1);
    std::random_device seeder;
    std::mt19937 engine(seeder());
    //float dist = u + dist_randn(engine);
    //float angle = th + th_randn(engine);

    float dist = u;
    float angle = th;

    float dx = dist*cos(angle) + dx_randn(engine);
    float dy = dist*sin(angle) + dy_randn(engine);

    for(auto p = this->particle_list.begin();p!= this->particle_list.end();p++)
    {
        p->x -= dx;
        p->y -= dy;
        //std::cerr<<"weight predicted "<<p->weight<<std::endl;
    }
}


void filter::update(std::vector<particle> measure,float Rx,float Ry)
{
    //update the particle weight with respect of the measure 
    // Rx is the x variance resp Ry 
    // hypth all the measure have the same pdf (moment etc )
    int i = 0;
    float closer_dist;
    float summ_weight=0;

    std::normal_distribution<float> mx_randn(0,Rx);
    std::normal_distribution<float> my_randn(0,Ry);
    std::random_device seeder;
    std::mt19937 engine(seeder());
    std::cerr<<"measure size "<<measure.size()<<std::endl;
    this->set_cluster_number(measure.size());
    for(auto p = this->particle_list.begin();p != this->particle_list.end();p++)
    {
        closer_dist = std::numeric_limits<float>::max();
        this->sumDist = 0;
        float closerX,closerY;
        for(particle m : measure)
        {
            float measure_x = m.x+mx_randn(engine);
            float measure_y = m.y+my_randn(engine);
            float dist = sqrt( pow(measure_x - p->x,2)+pow(measure_y-p->y,2));
            sumDist += dist;
            if(closer_dist>=dist)
            {
                closer_dist=dist;
                closerX = measure_x;
                closerY = measure_y;
            }
                
        }
        if(closer_dist == 0)
            closer_dist = 1e-6; // to avoid 0 div

        p->weight = 1.0/closer_dist;
        //p->weight *= this->probMesure(closerX,closer_dist,Rx)+this->probMesure(closerY,closer_dist,Ry);
        summ_weight += p->weight;

    }

    for(auto p = this->particle_list.begin();p != this->particle_list.end();p++)
        p->weight /= summ_weight;

}

void filter::resample(std::string method)
{
    
    std::vector<particle> new_particle_list;
    double p;
    int j;
    double tmp;
    double size = this->particle_list.size();

    int sizeParticleMeaningfull;
    std::normal_distribution<float> P(0,0.1); //var is 0.03m
    std::random_device seeder;
    std::mt19937 engine(seeder());
    float weightKeeped = 0;
    if(method=="naive") //need to take the cluster number into account
    {       double min = 1.1;
            double max = 0;
            double treshold;
            for(particle p:this->particle_list)
            {
                tmp += p.weight;
                if(min>p.weight)
                    min = p.weight;
                if(max < p.weight)
                    max = p.weight;

            }

            treshold = (max-min)/2; // 1/size // adpat treshold with 1/meandistance
            float meanX=0,meanY=0;
            //treshold = sumDist;
            for(int i=0;i <size ;i++)
            {   
                tmp = this->particle_list[i].weight;
                if(tmp>=treshold)
                {
                    new_particle_list.push_back(this->particle_list[i]);
                    meanX += this->particle_list[i].x;
                    meanY += this->particle_list[i].y;
                }
                
            }
            meanX /=  new_particle_list.size();
            meanY /= new_particle_list.size();
            sizeParticleMeaningfull = new_particle_list.size(); // we stock the meaningfull list size
            int numberParticleToCreate = size - sizeParticleMeaningfull;
            for(int i = 0;i<numberParticleToCreate;i++)
            {
                particle p = new_particle_list[i%sizeParticleMeaningfull];
                p.x = meanX + P(engine);
                p.y = meanY + P(engine);
                new_particle_list.push_back(p);
            }

    }else if("multimodiall")
    {
        std::vector<double> cum_sum;
        std::uniform_real_distribution<double> P(0.0,1.0);
        std::random_device seeder;
        std::mt19937 engine(seeder());
        
        for(int i=0;i < this->particle_list.size();i++)
        {
            if(i==0)
                cum_sum.push_back(this->particle_list[i].weight);
            else
                cum_sum.push_back(this->particle_list[i].weight + cum_sum[i-1]);
        }
    
        for(int i=0;i<int(this->particle_list.size()*0.9);i++)
        {
            p = P(engine);
            j=0;
            for(double c: cum_sum)
            {
                if(p <= c)
                {
                    new_particle_list.push_back(this->particle_list[i]);
                    break;
                }
            }
        }

        sizeParticleMeaningfull = new_particle_list.size(); // we stock the meaningfull list size
        int numberParticleToCreate = size - sizeParticleMeaningfull;
        for(int i = 0;i<numberParticleToCreate;i++)
        {
            particle p = new_particle_list[i%sizeParticleMeaningfull];
            p.x += P(engine);
            p.y += P(engine);
            p.weight = (1-weightKeeped)/numberParticleToCreate;
            new_particle_list.push_back(p);
        }
    }
    //std::cerr<<"old particle size "<<this->particle_list.size()<<" new particle size "<<new_particle_list.size()<<std::endl;
    this->particle_list = new_particle_list;
//    for(auto part :new_particle_list)
//        std::cerr<<"keeped particle "<<part.weight<<std::endl;

}

void filter::estimate()
{
    std::vector<float> meanClusterX;
    std::vector<float> meanClusterY;
    std::vector<float> varClusterX;
    std::vector<float> varClusterY; 
    std::vector<int> NCluster; //particle number in a cluster
    std::vector<float> weightedSumm;

    for(int i=0;i<this->clusterNumber;i++)
    {
        meanClusterX.push_back(0.0);
        meanClusterY.push_back(0.0);
        varClusterX.push_back(0.0);
        varClusterY.push_back(0.0);
        weightedSumm.push_back(0.00);
        NCluster.push_back(0.0);
        
    }

    for(particle p : this->particle_list)
    {
        
        int clusterID = p.cluster;
        float weight=1;//p.weight
        //std::cerr<<"weight "<<weight<<std::endl;
        NCluster[clusterID] += 1;
        meanClusterX[clusterID] += weight*p.x;
        meanClusterY[clusterID] += weight*p.y;
    }

    for(int i=0;i<this->clusterNumber;i++)
    {
        meanClusterX[i] = meanClusterX[i]/NCluster[i];
        meanClusterY[i] = meanClusterY[i]/NCluster[i];
    }

    std::cerr<<"mean cluser x "<<meanClusterX[0]<<std::endl;

    for(particle p : this->particle_list)
    {
        int clusterID = p.cluster;
        float weight = p.weight;
    
        varClusterX[clusterID] += weight*pow(p.x-meanClusterX[clusterID],2);
        varClusterY[clusterID] += weight*pow(p.y-meanClusterY[clusterID],2);
        weightedSumm[clusterID] += weight;
    }
    for(int i=0;i<this->clusterNumber;i++)
    {
        varClusterX[i] = meanClusterX[i]/NCluster[i];
        varClusterY[i] = meanClusterY[i]/NCluster[i];
        std::cerr<<"cluster i : "<<i<<" variance : "<<varClusterX[i]<<" , "<<varClusterY[i]<<" mean : "<<meanClusterX[i]<<" , "<<meanClusterY[i]<<std::endl;
        estimatedX.push_back(meanClusterX[i]);
        estimatedY.push_back(meanClusterY[i]);
    }
    std::cerr<<"size estimated x "<<estimatedX.size()<<std::endl;   
}

float filter::particleDistance(particle ps,particle pt)
{
    return sqrt( pow(ps.x-pt.x,2)+pow(ps.y-pt.y,2));
}

void filter::kMeansClustering(int epoch,int k)
{
    std::vector<particle> centroid;
    std::uniform_int_distribution<int> P(0.0,this->particle_list.size()-1);
    std::random_device seeder;
    std::mt19937 engine(seeder());
    std::vector<int> nPoints;
    std::vector<float> sumX, sumY;
    // centroid initialisation 
    for(int i=0;i<k;i++)
    {
        int index = P(engine);
        std::cerr<<"x init centroid "<<this->particle_list[i].x<<std::endl;
        centroid.push_back(this->particle_list[i]);
    }
    
    for(int z=0;z<epoch;z++)
    {
        //assigning particle to a cluster
        for(std::vector<particle>::iterator c = centroid.begin();c != centroid.end();c++)
        {
            int clusterID = c - begin(centroid);

            for(std::vector<particle>::iterator it = this->particle_list.begin();it!= this->particle_list.end();it++)
            {
                particle p  = *it;
                float dist = this->particleDistance(*c,p);
                if(dist <= p.min_dist)
                {
                    p.min_dist = dist;
                    p.cluster = clusterID;                
                }
                *it = p;
            }
        }   // choose centroid as measure and that should be fine with only steps above

        for (int j = 0; j < k; ++j) 
        {
            nPoints.push_back(0);
            sumX.push_back(0.0);
            sumY.push_back(0.0);
        }
        // get data for new clustering calculation
        for(std::vector<particle>::iterator it = this->particle_list.begin();it!= this->particle_list.end();it++)
        {
            int index = it->cluster; 
            nPoints[index] +=1;
            sumX[index] +=it->x;
            sumY[index] +=it->y;
            it->min_dist = std::numeric_limits<float>::max();
        }

        for(std::vector<particle>::iterator c = centroid.begin();c!=centroid.end();c++)
        {
            int index = c-begin(centroid);
            c->x = sumX[index]/nPoints[index];
            c->y = sumY[index]/nPoints[index];
        }
    for(auto p : this->particle_list)
    {
        std::cerr<<"cluster : "<<p.cluster<<std::endl;
    }

    
    
    }
    

/*
    std::vector<float> x,y;
    std::cerr<<"centroid size "<<centroid.size()<<std::endl;
    for(auto p : centroid)
    {
        std::cerr<<"x value "<<p.x<<std::endl;
        x.push_back(p.x);
        y.push_back(p.y);
    }
    plt::xlim(this->xmin-1,this->xmax+1);
    plt::ylim(this->ymin-1,this->ymax+1);
    std::cerr<<"x size "<<x.size()<<std::endl;
    plt::scatter(x,y);
    //plt::legend();
*/
}

void filter::set_cluster_number(int k)
{
    this->clusterNumber = k;
}    


float filter::calcNeff()
{
    float res=0;
    for(auto p : this->particle_list)
        res += p.weight*p.weight;
    return 1/res; 
}


int filter::get_size()
{
    return this->particle_list.size();
}

void filter::showEstimate()
{
    std::vector<float> x,y;
    x.push_back(this->estimatedX[this->estimatedX.size()-1]);
    y.push_back(this->estimatedY[this->estimatedY.size()-1]);
    std::map<std::string, std::string> keywords;
    keywords.insert(std::pair<std::string, std::string>("color", "red") );
    keywords.insert(std::pair<std::string, std::string>("marker", "x") );
    plt::scatter(x,y,10,keywords);
    std::cerr<<"size x : "<<x.size()<<std::endl;
}

void filter::resetWeight()
{
    float size = this->particle_list.size();
    for(auto it = this->particle_list.begin();it!=this->particle_list.end();it++)
    {
        it->weight = 1.0/size;
    }

}

float filter::probMesure(float z,float mean, float var)
{
    return 1/sqrt(2*M_PI*var)*exp(-(z-mean)*(z-mean)/(var));
}

void showMeasure(  std::vector<particle> tree_position)
{
    std::vector<float> x,y;
    std::map<std::string, std::string> keywords;
    for(auto t : tree_position)
    {
        x.push_back(t.x);
        y.push_back(t.y);
    }
    keywords.insert(std::pair<std::string, std::string>("color", "black") );
    keywords.insert(std::pair<std::string, std::string>("marker", "^") );
    plt::scatter(x,y,30,keywords);

    
}
int main() {

    std::vector<double> x,y;
    
    std::vector<particle> tree_position;
    particle tmp;
    tmp.x = 3;
    tmp.y = 5;
    tmp.weight = 0;
    tree_position.push_back(tmp);
    tmp.x = 0;
    tmp.y = 0;
    tmp.weight = 0;
    //tree_position.push_back(tmp);

    filter f;
    f.init();
    f.estimate();
    float t=1;
    //std::cerr<<" p("<<z<<") = "<<f.probMesure(z,0,1)<<std::endl;
    

    plt::pause(2);
    for(int i=0;i<16;i++)
    {   
        
        //f.estimate();
        plt::clf();
        plt::xlim(-7,7);
        plt::ylim(-7,7);
        plt::title("particular filter evolution" +  std::to_string(i));

        if( !(i%5))
        {
            f.predict(1,1.57);
            for(auto it=tree_position.begin();it!=tree_position.end();it++)
            {
                it->y -= 1;
            }
            //f.showParticle("prediction"," ");
        }
        f.update(tree_position,0.01,0.01);

        
        f.showParticle("base"," ");
        showMeasure(tree_position);
        f.showEstimate();
        plt::pause(t);
        plt::clf();
        plt::xlim(-7,7);
        plt::ylim(-7,7);
        std::cerr<<"Neff "<<f.calcNeff();
        //if(f.calcNeff()<f.get_size())
            f.resample("naive");
            f.showParticle(" "," ");
            showMeasure(tree_position);

            f.resetWeight();
        std::cerr<<"Neff after resample"<<f.calcNeff();
        //f.showParticle("resample"," ");

        f.estimate();

        plt::pause(t);
        
    }
    plt::show();
    f.estimate();
    return 0;
}