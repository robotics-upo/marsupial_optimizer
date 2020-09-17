/***********************************************************************/
/**     BISECTION NUMERICAL METHOD TO GET CATENARY CHAIN 3D POINTS     */
/**                                                                    */
/** Open Source Code                                                   */
/** Service Robotics Lab.            http://robotics.upo.es            */ 
/**                                  https://github.com/robotics-upo   */
/**                                                                    */
/** Maintainer:                     Contact:                           */
/** Simon Martinez Rozas            simon.martinez@uantof.cl           */
/**                                                                    */   
/***********************************************************************/
#ifndef _BISECTION_CATENARY_3D_H_
#define _BISECTION_CATENARY_3D_H_

#include <cmath>
#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>


using namespace std;

class bisectionCatenary
{
    public:
        struct points_interval
        {
            double pa;
            double pb;
        };
        vector <points_interval> vector_sign_change;

        struct points_catenary_2D
        {
            double x_;
            double y_;
        };
        vector <points_catenary_2D> catenary_chain_points_2D;

        struct points_catenary_3D
        {
            double x_;
            double y_;
            double z_;
        };
        vector <points_catenary_3D> catenary_chain_points_3D;

        bisectionCatenary();
        // ~bisectionCatenary();

        virtual double phi(double x);
        virtual double distanceC(double p); //Calculate distance from floor to point C
        virtual double catenaryPointA(double x);
        virtual double catenaryPointB(double y);
        virtual double evaluteCatenaryChain(double x_);
        virtual void integrateCatenaryChain2D();
        virtual double resolveBisection(double a, double b, int mode_);// 0 = to find phi() , 1 = to find X0 , 2 = to find Y0
        //Find points with sign changes in interval a-b, times that function pass through the origin 
        virtual void signChange(double a, double b, int mode_);
        virtual double computeFunction(double xr_aux, int mode_);
        virtual void getPointCatenary3D(vector<geometry_msgs::Point> &_v_p);
        virtual void directionVectorAxes();
        virtual void configBisection(double _l, double _x1, double _y1, double _z1, double _x2, double _y2, double _z2, int _id);
        virtual void resetVariables();
        virtual void setNumberPointsCatenary(int _n_p);
        virtual void setFactorBisection(double _fa,double _fb);

        //Length Catenary Chain 
        double L; 
        //Lashing points(X1,Y1,Z1) and (X2,Y2,Z2)
        double X1,Y1,Z1,X2,Y2,Z2;
        //Distance between lashing points in Axes X and Y represented in 2D plane
        double XB,YB;    
    // printf("enter setNumberPointsCatenary\n");
        double factor_bisection_a,factor_bisection_b;

        geometry_msgs::Point lower_point_3d_catenary;

    protected:

    double straight_tolerance;
    double kConst;

    double Ap, Bp; //Xtreme values Interval to evaluate Phi() Function 
    double Ax, Bx; //Xtreme values Interval to evaluate Catenary_A() Function
    double Ay, By; //Xtreme values Interval to evaluate Catenary_B() Function   
    double tolerance;
    //To save solutions of numeric method in function
    double bs_p, bs_Y0, bs_X0;
    int n_points, n_chain;
    double c_value, h_value, Xc, Yc, XC, YC, ZC;
    double direc_x , direc_y, direc_z, distance_3d ; 

    bool x_const, y_const, z_const;
    int id_vertex;

};



#endif
