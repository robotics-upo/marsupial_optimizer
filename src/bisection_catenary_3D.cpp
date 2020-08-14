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

#include "marsupial_g2o/bisection_catenary_3D.h"

//////////////////////////////////////////////////////////////////////////
//Implementation explanation:                                           //
//                                                                      //
//The implemented code get the 3D points position of a CHAIN that fallow//
//catenary. For that, Bisection Numerical Method is used to obtaining   //
//the value of the parameters that describe this physical phenomenon.   //
//                                                                      //
//In physics and geometry, a catenary is the curve that an idealized    // 
//hanging chain or cable assumes under its own weight when supported    //
//only at its ends.                                                     //
//                                                                      //
//In this case it is assumed that the lashing points (X1,Y1,Z1) and     //                                                                     
//(X2,Y2,Z2), which are not in the same height, and the length (L) of   //
//the cable are known.                                                  //
//                                                                      //
//To simplify, this problem is analyzed in 2D, considering that the     //
//chain is projected in a plane, and finally, de 2D points obtained from//
//bisection method are converted to 3D.                                 //
//                                                                      //
//Catenary equation referred to any axes whatever:                      //
//                      y-y0 = c* cosh ((x-x0)/c),                      //
//thus, for this equation to be defined, it is necessary to obtain 3    //
//constants: c, x0 and y0.                                              //
//                                                                      //
//The first equation is the condition of the catenary passing through   //
//point A:                                                              //
//        [i]           -y0 = c* cosh (x0/c)                            //
//                                                                      //
//The second equation is the condition of the catenary passing through  //
//point B:                                                              //
//        [ii]         yB-y0 = c* cosh ((xB-x0)/c)                      //
//                                                                      //
//The third and last equation will be the length of the cable arc.      //
//        [iii]    Sab = c * sin(X0/c)+ c*sin((xB-x0)/c)                //
//                                                                      //
//Then, we subtract, from the second equation, the first equation, and  //
//to this result, we will add the third equation (cable length):        //
//Sab+yB = c*cosh((xB-x0)/c)-c*cosh (x0/c)+c*sin(X0/c)+c*sin((xB-x0)/c) //
//                                                                      //
//Using trigonometry:                                                   //
//  Sab+yB = c*e^((xB-X0)/c) - c*e^(-x0/c) = c*e^(-x0/c)*(e^(xB/c)-1)   //
//                                                                      //
//Similar result is obtain if operation is subtaction:                  //
//  Sab-yB = c*e^((xB-X0)/c) + c*e^(-x0/c) = c*e^(x0/c)*(1+e^(-xB/c))   //
//                                                                      //
//Multiplaying the last two ecuations:                                  //
//               Sab^2-yB^2= c^2*4*[sinh(xB/(2*c))]^2                   //
//                                                                      //
//               sqrt(Sab^2-yB^2)= c*2*sinh(xB/(2*c))                   //
//         sqrt(Sab^2-yB^2) / xB = sinh(xB/(2*c)) / (xB/(2*c))          //
//                                                                      //
//Making the following variable change:   phi = xB/(2*c)                //
//                                          k = sqrt(Sab^2-yB^2)/xB     //
//Then we obtain the first ecuation to resolve by Bisection method:     //
//                        k*phi = sinh(phi)    , then we found c.       //
//                                                                      //
//Finally x0 is found by subtracting [i] and [ii] to eliminate Y0, and  //
//the bisection method is applied to the resulting equation.            //
//The value y0 is found using bisection method in [i].                  //
//                                                                      //
//                                                                      //
//////////////////////////////////////////////////////////////////////////


bisectionCatenary::bisectionCatenary()
{
    ROS_INFO("INITIALIZE BISECTION METHOD TO CALCULATE CATENARY CHAIN !!");
    // n_chain = 0.0;  //Number of points in catenary chain to get

} 

// bisectionCatenary::~bisectionCatenary(){} 


void bisectionCatenary::configBisection(double _l, double _x1, double _y1, double _z1, double _x2, double _y2, double _z2)
{
    resetVariables();
    if (n_chain == 0.0)
        ROS_ERROR("NOT SETING NUMBER OF POINTS TO CALCULATE IN CATENARY !!");
    
    L =_l;
    X1 =_x1; Y1 = _y1; Z1= _z1;
    X2 = _x2; Y2 = _y2;  Z2= _z2;
    
    distance_3d = sqrt(pow(X2-X1,2)+pow(Y2-Y1,2)+pow(Z2-Z1,2)); //Distance between lashing represent in 3D
    // printf("\nL=[%f] distance_3d=[%f] P1=[%f %f %f] P2=[%f %f %f]\n",L,distance_3d,X1,Y1,Z1,X2,Y2,Z2);
   
    XB = sqrt(pow(X2-X1,2)+pow(Y2-Y1,2));         //Distance between lashing represent in 2D plane
    YB = Z2 - Z1;    //height difference between points

    //Value to accurete solution
    straight_tolerance = 0.000001; //minimum difference between Sab and L to not consider the chain in a straight state
    tolerance = 0.0001; //
    n_points = 20;  // Number of interval to use in bisection method
        
    Ap =0.00001;    Bp = 4;
    Ax =-1000;          Bx = 1000;
    Ay =-1000;       By = 1000;

    //CALCULATE OF CATENARY POINTS
    if (distance_3d > L)
        ROS_ERROR("LENGTH OF CHAIN ​​IS SHORTER THAN 3D DISTANCE BETWEEN POINTS. Try other points or change L value !!!");

    //Calculate K for Phi()
    kConst = (sqrt(pow(L,2) - pow(YB,2)))/(XB); 
    if ( fabs(kConst) < straight_tolerance)
        ROS_WARN ("\nTHE CHAIN IS STRAIGHT!! NOT IN CATENARY STATE \n");
    
    directionVectorAxes();
    //Look for the solutions c, x0, y0
    //Calculate of phi and c
    bs_p = resolveBisection(Ap, Bp,0);
    c_value = distanceC(bs_p);
    //Calculate of x0
    bs_X0 = resolveBisection(Ax,Bx,1);
    //Calculate of y0
    bs_Y0 = resolveBisection(Ay,By,2);
    h_value = fabs(bs_Y0)-c_value;

    //Calculate lower point in catenary
    Xc=bs_X0;
    Yc=Z1-h_value;

    //Calculate points for Catenary chain
    integrateCatenaryChain();
}

void bisectionCatenary::directionVectorAxes()
{
    if (X2 > X1)
        direc_x = 1.0;
    if (X2 < X1)
        direc_x = -1.0;
    if (X2 == X1)
    {
        direc_x = 0.0;    
        // printf("\nChain perperdicular with Axes X\n");
    }
    if (Y2 > Y1)
        direc_y = 1.0;
    if (Y2 < Y1)
        direc_y = -1.0;
    if (Y2 == Y1)
    {
        direc_y = 0.0;    
        // printf("\nChain perperdicular with Axes Y\n");
    }
}

double bisectionCatenary::phi(double x) 
{
    return (sinh(x)- kConst*x);
}

double bisectionCatenary::distanceC(double p)
{
    double result_ = (XB)/(2*p);
    // printf("\nValue of 'c = %g'\n",result_);
    return result_;
}

double bisectionCatenary::catenaryPointA(double x)
{
    return (c_value*cosh(((XB)-x)/c_value) - c_value*cosh((x)/c_value) - YB);
    // return (c_value*cosh(((XB)-x)/c_value) - c_value*cosh((X1-x)/c_value) - YB + Y1);
}

double bisectionCatenary::catenaryPointB(double y)
{
    return (c_value*cosh(((XB)-bs_X0)/c_value) - (YB) + y);
}

double bisectionCatenary::evaluteCatenaryChain(double x_)
{
    return (c_value * cosh((x_- Xc)/c_value)+ (Yc-c_value));
}

void bisectionCatenary::integrateCatenaryChain()
{
    double x_value, y_value, x_step;
    // int pos_lower_point;
    // double Ymin = Z1;
    points_catenary_2D point_cat;
    catenary_chain_points_2D.clear();
    x_value = 0;

    x_step = (XB)/n_chain;

    for (int i=0 ; i < n_chain +1 ; i ++)
    {
        y_value = evaluteCatenaryChain(x_value);
        point_cat.x_ = x_value;
        point_cat.y_ = y_value;
        // printf("Points Catenary x_= %f , y_= %f\n",x_value,y_value);
        catenary_chain_points_2D.push_back(point_cat);
        x_value = x_value + x_step;
    
        // if (Ymin > y_value)
        // {
        //     Ymin = y_value;
        //     pos_lower_point = i;
        // }
    }
    // printf("Vector 'catenary_chain_points_2D' size = %i\n",catenary_chain_points_2D.size());
    // printf("\nFOUND IT!! The lowest point Catenary 2D Xc= %f , Yc= %f\n",
    // catenary_chain_points_2D[pos_lower_point].x_,catenary_chain_points_2D[pos_lower_point].y_);
}

double bisectionCatenary::modeBisection(double xr_aux, int mode_)
{
    double yr_aux;
    
    if (mode_ == 0)
    {
        yr_aux = phi(xr_aux);
    }
    if (mode_ == 1)
    {
        yr_aux = catenaryPointA(xr_aux);
    }
    if (mode_ == 2)
    {
        yr_aux = catenaryPointB(xr_aux);  
    }
    return yr_aux;
}

double bisectionCatenary::resolveBisection(double a, double b,int mode_) 
{
    double xr, error;
    //Initialize error with a big value
    error = tolerance+1;
    
    double x_a , x_b;
    points_interval it_points_interval;
    // printf ("\nbisection mode = %i",mode_);
    // printf ("\nLooking for Function roots:\n");
    signChange(a, b, mode_);

    for (unsigned i = 0; i < vector_sign_change.size(); i++)
    {
        it_points_interval = vector_sign_change[i];
        x_a = it_points_interval.pa;
        x_b = it_points_interval.pb;
        
        while (error > tolerance) 
        {
            xr = (x_a + x_b) / 2.0;
            if ((modeBisection(x_a,mode_) * modeBisection(xr,mode_) < 0) || 
                (modeBisection(x_a,mode_) * modeBisection(xr,mode_) == 0))
            {
                x_b = xr;
            }
            else if ((modeBisection(xr,mode_) * modeBisection(x_b,mode_)< 0) ||
                (modeBisection(x_b,mode_) * modeBisection(xr,mode_) == 0))
            {
                x_a = xr;
            }
            error = fabs(modeBisection(xr,mode_));
            if (error < tolerance)
            {
                if (mode_ == 0){
                    // printf("SOLUTION FOUNDED!!! for 'PHI(phi) = %f'  'phi = %f'\n",modeBisection(xr,mode_),xr);
                }
                if (mode_ == 1){
                    // printf("SOLUTION FOUNDED!!! for 'CatenaryA(X0) = %f'  'X0 = %f'\n",modeBisection(xr,mode_),xr);
                }
                if (mode_ == 2){
                    // printf("SOLUTION FOUNDED!!! for 'CatenaryB(Y0) = %f'  'Y0 = %f'\n",modeBisection(xr,mode_),xr);
                }
            }
        }
    }
    return xr;
}

void bisectionCatenary::signChange(double a, double b, int mode_)
{
    points_interval points_sign_change;
    vector_sign_change.clear();
    
    double y1 , y2;
    double interval = fabs(b - a) / n_points;
    double bound_a = a;
    double bound_b;
    
    // printf ("signChange mode = %i\n",mode_);

    for (int i = 0; i < n_points; i++) 
    {
        y1 = modeBisection(bound_a,mode_);
        bound_b = bound_a + interval;
        y2 = modeBisection(bound_b,mode_);
        
        if ((y1*y2 < 0) || (y1*y2 == 0))
        {
            points_sign_change.pa = bound_a;
            points_sign_change.pb = bound_b;
            vector_sign_change.push_back(points_sign_change);
        }           
        bound_a = bound_b;
    }
    // printf("vector size = %i\n",vector_sign_change.size());

    for (unsigned i = 0; i < vector_sign_change.size(); i++)
    {  
        points_interval print_pts_;
        print_pts_.pa = vector_sign_change[i].pa;
        print_pts_.pb = vector_sign_change[i].pb;
        if (mode_ == 0){
            // ROS_INFO("Value point save for phi(x) xa = %g  xb= %g",print_pts_.pa,print_pts_.pb);
        }
        if (mode_ == 1){
            // ROS_INFO("Value point save for catenary_A(x) xa = %g  xb= %g",print_pts_.pa,print_pts_.pb);
        }
        if (mode_ == 2){
            // ROS_INFO("Value point save for Catenary_B(x) xa = %g  xb= %g",print_pts_.pa,print_pts_.pb);
        }
    }
    if (vector_sign_change.size() < 1 )
    {
        if (mode_ == 0)
            ROS_ERROR("PROBLEM WITHOUT SOLUTION FOR phi() !!! Interval [a-b] doesn't enclose a root");
        if (mode_ == 1)
            ROS_ERROR("PROBLEM WITHOUT SOLUTION FOR Catenary_A() !!! Interval [a-b] doesn't enclose a root");
        if (mode_ == 2)
            ROS_ERROR("PROBLEM WITHOUT SOLUTION FOR Catenary_B() !!! Interval [a-b] doesn't enclose a root");
    }
}

void bisectionCatenary::getPointCatenary3D(vector<geometry_msgs::Point> &_v_p)
{
    double tetha = atan(fabs(Y2-Y1)/fabs(X2-X1));
    double Zmin = Z1;
    int pos_lower_point;
    
    points_catenary_3D point_cat3D;

    catenary_chain_points_3D.clear();
    _v_p.clear();

    for(size_t i=0; i < catenary_chain_points_2D.size(); i++)
    {
        geometry_msgs::Point _p;

        point_cat3D.z_ = catenary_chain_points_2D[i].y_;
        point_cat3D.x_ = X1 + direc_x* cos(tetha) * catenary_chain_points_2D[i].x_;
        point_cat3D.y_ = Y1 + direc_y* sin(tetha) * catenary_chain_points_2D[i].x_;
        catenary_chain_points_3D.push_back(point_cat3D);
        if (Zmin > point_cat3D.z_)
        {
            Zmin = point_cat3D.z_;
            pos_lower_point = i;
        }
        _p.x = point_cat3D.x_;
        _p.y = point_cat3D.y_;
        _p.z = point_cat3D.z_;    
        _v_p.push_back(_p);
        // printf("Points Catenary X_= %f , Y_= %f , Z = %f\n",
        // catenary_chain_points_3D[i].x_,catenary_chain_points_3D[i].y_,catenary_chain_points_3D[i].z_);
    }
    // ROS_INFO("CATENARY FOUND IT!! The lowest point Catenary 3D X_= %f , Y_= %f , Z = %f",
    // catenary_chain_points_3D[pos_lower_point].x_,catenary_chain_points_3D[pos_lower_point].y_,catenary_chain_points_3D[pos_lower_point].z_);
}

void bisectionCatenary::resetVariables(){

    L = 0.0;

    X1 = 0.0; 
    Y1 = 0.0; 
    Z1 = 0.0;
    X2 = 0.0;
    Y2 = 0.0;  
    Z2 = 0.0;

    Xc= 0.0;
    Yc= 0.0;

    kConst = 0.0;

    bs_p = 0.0;
    c_value = 0.0;
    bs_X0 = 0.0;
    bs_Y0 = 0.0;
    h_value = 0.0;
}

inline void bisectionCatenary::setNumberPointsCatenary(int _n_p){
    // printf("enter setNumberPointsCatenary\n");
    n_chain = ceil(_n_p);
}
