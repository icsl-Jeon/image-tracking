#ifndef DBSCAN_H
#define DBSCAN_H

#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <cmath>
#include <map>

namespace DBSCAN{

const int NOISE = -2;
const int NOT_CLASSIFIED = -1;

class Point {
public:
    double x, y;
    int ptsCnt, cluster;
    double getDis(const Point & ot) {
        return sqrt((x-ot.x)*(x-ot.x)+(y-ot.y)*(y-ot.y));
    }
};

class DBCAN {
public:
    int n, minPts;
    double eps;
    std::vector<Point> points;
    int size;
    std::vector<std::vector<int> > adjPoints;
    std::vector<bool> visited;
    std::vector<std::vector<int> > cluster;
    int clusterIdx;

    DBCAN(int n, double eps, int minPts, std::vector<Point> points) {
        this->n = n;
        this->eps = eps;
        this->minPts = minPts;
        this->points = points;
        this->size = (int)points.size();
        adjPoints.resize(size);
        this->clusterIdx=-1;
    }
    void run () {
        checkNearPoints(); //adjoint matrix

        for(int i=0;i<size;i++) {
            if(points[i].cluster != NOT_CLASSIFIED) continue;

            if(isCoreObject(i)) { // does this pnt have enough connected points?
                dfs(i, ++clusterIdx);
            } else {
                points[i].cluster = NOISE;
            }
        }

        cluster.resize(clusterIdx+1);
        for(int i=0;i<size;i++) {
            if(points[i].cluster != NOISE) {
                cluster[points[i].cluster].push_back(i);
            }
        }
    }

    void dfs (int now, int c) {
        points[now].cluster = c;
        if(!isCoreObject(now)) return;

        for(auto&next:adjPoints[now]) {
            if(points[next].cluster != NOT_CLASSIFIED) continue;
            dfs(next, c);
        }
    }

    void checkNearPoints() {
        for(int i=0;i<size;i++) {
            for(int j=0;j<size;j++) {
                if(i==j) continue;
                if(points[i].getDis(points[j]) <= eps) {
                    points[i].ptsCnt++;
                    adjPoints[i].push_back(j);
                }
            }
        }
    }
    // is idx'th point core object?
    bool isCoreObject(int idx) {
        return points[idx].ptsCnt >= minPts;
    }

    std::vector<std::vector<int> > getCluster() {
        return cluster;
    }

    std::vector<std::vector<double>> getClusterCenter(){
        std::vector<std::vector<double>> cluster_centers;

        for(int i=0;i<cluster.size();i++) //per cluster
        {
            double cur_cluster_center_x=0;
            double cur_cluster_center_y=0;

            for(int j=0;j<cluster[i].size();j++)

            {   cur_cluster_center_x+=points[cluster[i][j]].x;
                cur_cluster_center_y+=points[cluster[i][j]].y;
            };


            cur_cluster_center_x=cur_cluster_center_x/cluster[i].size();
            cur_cluster_center_y=cur_cluster_center_y/cluster[i].size();

            std::vector<double> cur_center;
            cur_center.reserve(2);
            cur_center.push_back(cur_cluster_center_x);
            cur_center.push_back(cur_cluster_center_y);

            cluster_centers.push_back(cur_center);

        }

        return cluster_centers;
    }

};


/**
int main(int argc, const char * argv[]) {
    if(argc!=5) {
        cout << "Please follow this format. clustering.exe [intput] [n] [eps] [minPts]";
        return 0;
    }

    string inputFileName(argv[1]);
    string n(argv[2]);
    string eps(argv[3]);
    string minPts(argv[4]);

    InputReader inputReader(inputFileName);

    DBCAN dbScan(stoi(n), stod(eps), stoi(minPts), inputReader.getPoints());
    dbScan.run();

    OutputPrinter outputPrinter(stoi(n), inputFileName, dbScan.getCluster());
    outputPrinter.print();

    return 0;
}
**/
}

#endif
