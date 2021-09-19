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

        vector <geometry_msgs::Point> catenary_chain_points_3D;

        bisectionCatenary();
        // ~bisectionCatenary();

        virtual bool configBisection(double _l, double _x1, double _y1, double _z1, double _x2, double _y2, double _z2);
        virtual double resolveBisection(double a1_, double b1_, int mode_);// 0 = to find phi() , 1 = to find X0 , 2 = to find Y0
        //Find points with sign changes in interval a-b, times that function pass through the origin 
        virtual bool lookingSignChanging (double a, double b, int mode_);
        virtual double functionBisection(double xr_aux, int mode_);
        virtual void getPointCatenary3D(vector<geometry_msgs::Point> &_v_p);
        virtual void getPointCatenaryStraight(vector<geometry_msgs::Point> &_v_p);
        virtual void resetVariables();
        virtual void setFactorBisection(double _fa,double _fb);
        virtual void checkStateCatenary(double _x1, double _y1, double _z1, double _x2, double _y2, double _z2);
        virtual void getNumberPointsCatenary(double _length);
        virtual bool setNumPointsPerUnitLength(int n);
        virtual void setResolution(int res_);
        virtual void getMinPointZCat(geometry_msgs::Point &p_, int &n_);

        //Length Catenary Chain 
        double L; 
        //Lashing points(X1,Y1,Z1) and (X2,Y2,Z2)
        double X1,Y1,Z1,X2,Y2,Z2;
          
        double factor_bisection_a, factor_bisection_b;

        geometry_msgs::Point min_point_z_cat;
        int pos_in_cat_z_min;

    protected:
        double kConst;

        double XB,YB; //Distance between lashing points in Axes X and Y represented in 2D plane

        double Ap, Bp; //Xtreme values Interval to evaluate Phi() Function 
        double Ax, Bx; //Xtreme values Interval to evaluate Catenary_A() Function
        double Ay, By; //Xtreme values Interval to evaluate Catenary_B() Function   
        double tolerance;
        //To save solutions of numeric method in function
        double bs_p, bs_Y0, bs_X0;
        double n_interval; 
        int num_point_catenary;
        int num_point_per_unit_length;
        double c_value, h_value, Xc, Yc, XC, YC, ZC;
        double _direc_x , _direc_y, _direc_z, distance_3d ; 

        int div_res;
        double resolution;

        bool x_const, y_const, z_const;

        bool interval_finded;
};

#endif
