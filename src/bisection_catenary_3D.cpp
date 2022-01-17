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

#include "misc/bisection_catenary_3D.h"

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
    num_point_per_unit_length = 10;
    resolution = 0.05;
    div_res = 1.0/resolution;
    factor_bisection_a = 1000.0;
    factor_bisection_b = 1000.0;
    first_coll = 0;
    last_coll = 0;
    received_grid = false;
    get_distance_data = false;

    use_markers = false;
} 

bisectionCatenary::bisectionCatenary(ros::NodeHandlePtr nhP_)
{
    num_point_per_unit_length = 10;
    resolution = 0.05;
    div_res = 1.0/resolution;
    factor_bisection_a = 1000.0;
    factor_bisection_b = 1000.0;
    first_coll = 0;
    last_coll = 0;
    received_grid = false;
    get_distance_data = false;
    nhP = nhP_;

    use_markers = true;

	points_between_cat_marker_pub_ = nhP->advertise<visualization_msgs::MarkerArray>("points_between_cat_marker", 1);
} 

// bisectionCatenary::~bisectionCatenary(){} 

bool bisectionCatenary::configBisection(double _l, double _x1, double _y1, double _z1, double _x2, double _y2, double _z2, bool get_distance_data_)
{
    resetVariables();
  
    get_distance_data = get_distance_data_;
    if ( (get_distance_data && !received_grid) || (!get_distance_data && received_grid) ){ 
        printf(PRINTF_RED" Warnning: you are traying to use Bisection Method using grid dintance, but grid object is still empty.");
        printf(PRINTF_RED" Check that you are calling readDataForCollisionAnalisys() method before call configBisection() method, and in the latter defined get_distance_data_ argument as true. \n");
        printf(PRINTF_REGULAR" \n");
    }

    L =_l;
    X1 =_x1; Y1 = _y1; Z1= _z1;
    X2 = _x2; Y2 = _y2;  Z2= _z2;

    L_minor_than_D = false;
    
    distance_3d = sqrt(pow(X2-X1,2)+pow(Y2-Y1,2)+pow(Z2-Z1,2)); //Distance between lashing represent in 3D
    
    XB = sqrt(pow(X2-X1,2)+pow(Y2-Y1,2));         //Distance between lashing represent in 2D plane
    YB = Z2 - Z1;    //height difference between points

    checkStateCatenary(_x1, _y1, _z1, _x2, _y2, _z2);
    if (L > distance_3d)
        getNumberPointsCatenary(L);
    else{
        getNumberPointsCatenary(1.001*distance_3d);
        printf("Warning: Input Length minor that necesary distance between point to apply bisection method\n");
        L_minor_than_D = true;
    }

    mid_p_cat = ceil(num_point_catenary/2.0);

    if (!x_const || !y_const){ 
        //Value to accurete solution
        tolerance = 0.00001; //
        n_interval = 50.0;  // Number of interval to use in bisection method
        Ap =0.0;    Bp = 10.0;
        Ax =-100.0;      Bx = 100.0;
        Ay =-100.0;      By = 100.0;
        
        //Calculate K for Phi()
        kConst = (sqrt( fabs(L*L - YB*YB) ))/(XB); 
        
        //Calculate of phi and c
        bs_p = resolveBisection(Ap, Bp,0);
        c_value = XB/(2.0*bs_p); // Calculate distance from floor to point C
        //Calculate of x0
        if (c_value > 1.0)
            factor_bisection_a = floor(c_value);
        else
            factor_bisection_a = c_value;
        bs_X0 = resolveBisection(-1.0*factor_bisection_a, factor_bisection_a,1);
        // Calculate of y0
        bs_Y0 = resolveBisection(-1.0*factor_bisection_a, factor_bisection_a,2);
        h_value = fabs(bs_Y0)-c_value;

        //Calculate lower point in catenary
        Xc = bs_X0;
        Yc = Z1-h_value;
        return false;
    }
    else
        return true;
}

void bisectionCatenary::checkStateCatenary(double _x1, double _y1, double _z1, double _x2, double _y2, double _z2)
{
  if( fabs(_x1 - _x2) <  0.001)
        x_const = true;
  if( fabs(_y1 - _y2) <  0.001)
        y_const = true;
  if( fabs(_z1 - _z2) <  0.001)
        z_const = true;
  if(x_const && y_const){
  }

  if(_x2 > _x1)
        _direc_x = 1.0;
  else if(_x2 < _x1)
        _direc_x = -1.0;
  else if(_x2 == _x1)
        _direc_x = 0.0;    
  
  if(_y2 > _y1)
        _direc_y = 1.0;
  else if(_y2 < _y1)
        _direc_y = -1.0;
  else if(_y2 == _y1)
        _direc_y = 0.0;    
}

void bisectionCatenary::getPointCatenary3D(vector<geometry_msgs::Point> &v_p_, bool dist_interpolation_)
{
    std::vector<geometry_msgs::Point> v_p_bet_cat;
    double p_z_min = 1000.0;
    dist_obst_cat.clear(); pos_cat_in_coll.clear(); cat_between_obs.clear();
    if (!x_const || !y_const){ 
        double x_value_, y_value_, x_step;
        // int check_continuity_ = 0;
        x_step = (XB)/num_point_catenary;
        x_value_ = 0.0;
        
        double tetha = atan(fabs(Y2-Y1)/fabs(X2-X1));

        v_p_.clear();

        double dist_cat_obs;
        geometry_msgs::Point p_;
        float p_x_, p_y_, p_z_;
        first_coll = last_coll = 0;

        min_distance_cat_obs = 1000.0;
        if (use_markers)
            clearMarkers(p_bet_cat_marker, 50, points_between_cat_marker_pub_);

        for(size_t i=0; i < num_point_catenary ; i++){
            y_value_ = (c_value * cosh((x_value_ - Xc)/c_value)+ (Yc - c_value)); // evalute CatenaryChain
            p_.x = X1 + _direc_x* cos(tetha) * x_value_;
            p_.y = Y1 + _direc_y* sin(tetha) * x_value_;
            p_.z = y_value_;
            p_x_ = resolution * round( (p_.x) * div_res);
            p_y_ = resolution * round( (p_.y) * div_res);
            p_z_ = resolution * round( (p_.z) * div_res);
            
            x_value_ = x_value_ + x_step;

            if(get_distance_data){
                bool is_into_ = grid_3D->isIntoMap(p_x_,p_y_,p_z_);
                if(is_into_){
                    if (dist_interpolation_){
                    TrilinearParams d = grid_3D->getPointDistInterpolation((double)p_x_, (double)p_y_, (double)p_z_);
                    double x = p_.x;
                    double y = p_.y;
                    double z = p_.z;
                    dist_cat_obs= (d.a0 + d.a1*x + d.a2*y + d.a3*z + d.a4*x*y + d.a5*x*z + d.a6*y*z + d.a7*x*y*z); 
                    }
                    else
                        dist_cat_obs =  grid_3D->getPointDist(p_x_,p_y_,p_z_);
                }
                else
                    dist_cat_obs = -1.0;
                dist_obst_cat.push_back(dist_cat_obs);

                if (min_distance_cat_obs > dist_cat_obs && min_distance_cat_obs > 0.0)
                    min_distance_cat_obs = dist_cat_obs;
                
                if( i>0 ){ //Check if there is obtacle between two catenary points
                    double d_p_cat = sqrt((p_.x-v_p_[i-1].x)*(p_.x-v_p_[i-1].x)+(p_.y-v_p_[i-1].y)*(p_.y-v_p_[i-1].y)+(p_.z-v_p_[i-1].z)*(p_.z-v_p_[i-1].z));
                    int n_p_bet_cat = round (d_p_cat/0.04);
                    double interval_xy = 0.0; 
                    double interval_z = 0.0; 
                    if (n_p_bet_cat > 0){
                        interval_xy = sqrt((p_.x-v_p_[i-1].x)*(p_.x-v_p_[i-1].x) + (p_.y-v_p_[i-1].y)*(p_.y-v_p_[i-1].y)) /n_p_bet_cat; 
                        interval_z = (p_.z-v_p_[i-1].z) /n_p_bet_cat; 
                    }
                    double tetha_ = atan2((p_.y-v_p_[i-1].y),(p_.x-v_p_[i-1].x));
                    v_p_bet_cat.clear();
                    // printf("n_p_bet_cat = %i , interval=[%f/%f] d_p_cat=%f , p_[%f %f %f] p_-1[%f %f %f] \n", n_p_bet_cat, interval_xy, interval_z, d_p_cat, p_.x, p_.y, p_.z,  v_p_[i-1].x,  v_p_[i-1].y,  v_p_[i-1].z);
                    
                    for (int j = 0; j < n_p_bet_cat + 1 ; j++){
                        geometry_msgs::Point p_b_cat_;
                        p_b_cat_.x = v_p_[i-1].x + cos(tetha_) * interval_xy*(j);
                        p_b_cat_.y = v_p_[i-1].y + sin(tetha_) * interval_xy*(j);
                        p_b_cat_.z = v_p_[i-1].z + (j)*(interval_z);
                        v_p_bet_cat.push_back(p_b_cat_);
                        if(j > 0){
                            // printf("\t v_p_bet_cat.size()=%lu p_b_cat_[%f %f %f]\n", v_p_bet_cat.size(), p_b_cat_.x, p_b_cat_.y, p_b_cat_.z);
                            double p_b_cat_x_ = resolution * round( (p_b_cat_.x) * div_res);
                            double p_b_cat_y_ = resolution * round( (p_b_cat_.y) * div_res);
                            double p_b_cat_z_ = resolution * round( (p_b_cat_.z) * div_res);
                            bool _is_into_ = grid_3D->isIntoMap(p_b_cat_x_, p_b_cat_y_, p_b_cat_z_);
                            double d_obt_cat;
                            if (_is_into_)
                                d_obt_cat = grid_3D->getPointDist((double)p_b_cat_.x,(double)p_b_cat_.y,(double)p_b_cat_.z);
                            else
                                d_obt_cat = -1.0;
                            double a_;
                            if(j==0)
                                a_ = 1.0;
                            else
                                a_ = 0.80;

                            octomap::point3d r_;
                            octomap::point3d s_(v_p_bet_cat[j-1].x,v_p_bet_cat[j-1].y,v_p_bet_cat[j-1].z); //start for rayCast
                            octomap::point3d d_(v_p_bet_cat[j].x-v_p_bet_cat[j-1].x,v_p_bet_cat[j].y-v_p_bet_cat[j-1].y,v_p_bet_cat[j].z-v_p_bet_cat[j-1].z); //direction for rayCast

                            bool r_cast_coll = false; 
                            r_cast_coll = octotree_full->castRay(s_, d_, r_);
                            double dist12_ = sqrt ( (v_p_bet_cat[j].x-s_.x())*(v_p_bet_cat[j].x-s_.x())+
                                                    (v_p_bet_cat[j].y-s_.y())*(v_p_bet_cat[j].y-s_.y())+ 
                                                    (v_p_bet_cat[j].z-s_.z())*(v_p_bet_cat[j].z-s_.z()) );
                            double distObs_ = sqrt ((s_.x()-r_.x())*(s_.x()-r_.x())+(s_.y()-r_.y())*(s_.y()-r_.y())+(s_.z()-r_.z())*(s_.z()-r_.z()) );
                            bool ray_cat_coll_ = false;
                            if(r_cast_coll && distObs_ <= dist12_)
                                ray_cat_coll_ = true;
                            through_obst.push_back(ray_cat_coll_);

                            if (d_obt_cat < bound_obst*a_ && ray_cat_coll_){
                            // if (d_obt_cat < bound_obst*a_){
                                cat_between_obs.push_back(i);	
                                if (first_coll == 0){
                                    first_coll  = i-1;   
                                    pos_cat_in_coll.push_back(first_coll); 
                                }
                                last_coll = i;
                                pos_cat_in_coll.push_back(i);
                                break;
                            }
                        }
                    }
                    if (use_markers) 
                        markerPoints(p_bet_cat_marker, v_p_bet_cat, points_between_cat_marker_pub_);
                }
                // }        
            }
          
            
            // if ( ((last_coll-first_coll) == check_continuity_ ) && check_continuity_ != 0 && first_coll != 0 && last_coll != 0)
            //     last_coll = num_point_catenary;

            v_p_.push_back(p_);
            if (p_z_min > p_.z){
                min_point_z_cat.x = p_.x; 
                min_point_z_cat.y = p_.y; 
                min_point_z_cat.z = p_.z; 
                pos_in_cat_z_min = i;
                p_z_min = p_.z;
            }
            if (mid_p_cat == i){
                mid_point_cat.x = p_.x;
				mid_point_cat.y = p_.y;
				mid_point_cat.z = p_.z;
            }
        }
        // for (size_t k  = 0 ; k < pos_cat_in_coll.size() ; k++){
        //     printf("pos_cat_in_coll[%lu]=%i \n",k, pos_cat_in_coll[k]);
        // }
    }
    else{
        v_p_.clear();
        getPointCatenaryStraight(v_p_);
    }
    
}

void bisectionCatenary::getPointCatenaryStraight(std::vector<geometry_msgs::Point> &v_p_)
{
    double _step = distance_3d / (double) num_point_catenary;

    for(int i=0; i < num_point_catenary ; i++)
    {       
        geometry_msgs::Point p_;

        p_.x = resolution * ( round(X1*div_res ));
        p_.y = resolution * ( round(Y1*div_res ));
        p_.z = resolution * ( round((Z1 + _step* (double)i)*div_res) );    
        v_p_.push_back(p_);
        // printf("point_cat = [%f %f %f]\n",p_.x, p_.y, p_.z);
    }
}

double bisectionCatenary::resolveBisection(double a1_, double b1_, int mode_) 
{
    double xr, error, xa_ , xb_;
    
    bool find_sign_change = false;
    double bound_interval_1 = (a1_);
    double bound_interval_2 = (b1_);
    while(!find_sign_change){    
        find_sign_change = lookingSignChanging(bound_interval_1, bound_interval_2, mode_);  // To find smallers interval where can be find the solution, depend on the sign change
		bound_interval_1 = bound_interval_1 * 2.0;
        bound_interval_2 = bound_interval_2 * 2.0;
    }
    
    for (unsigned i = 0; i < vector_sign_change.size(); i++){
        error = tolerance + 1.0;
        xa_ =  vector_sign_change[i].pa;
        xb_ =  vector_sign_change[i].pb;
        
        while (error > tolerance) {
            xr = (xa_ + xb_) / 2.0;
            if ( (functionBisection(xa_,mode_) * functionBisection(xr,mode_) < 0.0) || (functionBisection(xa_,mode_) * functionBisection(xr,mode_) == 0.0) )
                xb_ = xr;
            else if ( (functionBisection(xr,mode_) * functionBisection(xb_,mode_)< 0.0) || (functionBisection(xb_,mode_) * functionBisection(xr,mode_) == 0.0) )
                xa_ = xr;
            
            error = fabs(functionBisection(xr,mode_));
            // if (error < tolerance){
            //     if (mode_ == 0)
            //         printf("SOLUTION FOUNDED!!! for 'PHI(phi) = %f'  'phi = %f'\n",functionBisection(xr,mode_),xr);
            //     else if (mode_ == 1)
            //         printf("SOLUTION FOUNDED!!! for 'CatenaryA(X0) = %f'  'X0 = %f'\n",functionBisection(xr,mode_),xr);
            //     else if (mode_ == 2)
            //         printf("SOLUTION FOUNDED!!! for 'CatenaryB(Y0) = %f'  'Y0 = %f'\n",functionBisection(xr,mode_),xr);
            // }
        }
    }
    return xr;
}

double bisectionCatenary::functionBisection(double xr_aux, int mode_)
{
    double yr_aux;
    
    if (mode_ == 0)
        yr_aux = sinh(xr_aux)- kConst * xr_aux; // To calculate phi()
    if (mode_ == 1)
        yr_aux = c_value*cosh((XB-xr_aux)/c_value) - c_value*cosh(xr_aux/c_value) - YB; // To calculate catenaryPointA
    if (mode_ == 2)
        yr_aux = c_value*cosh(((XB)-bs_X0)/c_value) - (YB) + xr_aux; // To calculate catenaryPointB
    return yr_aux;
}

bool bisectionCatenary::lookingSignChanging (double a2, double b2, int mode_)
{
    points_interval points_sign_change;
    vector_sign_change.clear();
    
    double y1 , y2;
    double interval = fabs(b2 - a2) / n_interval;
    double bound_a = a2 ;
    double bound_b;
    
    for (int i = 0; i < n_interval; i++) {
        y1 = functionBisection(bound_a, mode_);
        bound_b = bound_a + interval;
        y2 = functionBisection(bound_b, mode_);

        if ((y1*y2 < 0.0) || (y1*y2 == 0.0)){
            points_sign_change.pa = bound_a;
            points_sign_change.pb = bound_b;
            vector_sign_change.push_back(points_sign_change);
        }           
        bound_a = bound_b;
    }

    if (vector_sign_change.size() < 1.0 )
        return false;
    else
        return true;
}

inline void bisectionCatenary::getNumberPointsCatenary(double _length){num_point_catenary = round( (double)num_point_per_unit_length * _length);}  

bool bisectionCatenary::setNumPointsPerUnitLength(int n)
{
    if(n>0)
    {
        num_point_per_unit_length = n;
        return true;
    } 
    else
        return false;
}

void bisectionCatenary::setResolution(int res_)
{
  resolution = res_;
  div_res = 1.0/resolution;
}

inline void bisectionCatenary::setFactorBisection(double _fa,double _fb){factor_bisection_a = _fa; factor_bisection_b = _fb;}

void bisectionCatenary::getMinPointZCat(geometry_msgs::Point &p_, int &n_)
{
    p_.x = min_point_z_cat.x;
    p_.y = min_point_z_cat.y;
    p_.z = min_point_z_cat.z;
    n_ = pos_in_cat_z_min;
}

void bisectionCatenary::getMidPointCat(geometry_msgs::Point &p_, int &n_)
{
    p_.x = mid_point_cat.x;
    p_.y = mid_point_cat.y;
    p_.z = mid_point_cat.z;
    n_ = mid_p_cat;
}

void bisectionCatenary::resetVariables()
{
    L = 0.0;
    X1 = 0.0; Y1 = 0.0; Z1 = 0.0;X2 = 0.0;Y2 = 0.0;  Z2 = 0.0;
    YB= 0.0; XB= 0.0; Xc= 0.0; Yc= 0.0;
    kConst = 0.0;
    bs_p = 0.0;
    c_value = 0.0;
    bs_X0 = 0.0;
    bs_Y0 = 0.0;
    h_value = 0.0;
    x_const = y_const = z_const = false;
    dist_obst_cat.clear();
}

void bisectionCatenary::readDataForCollisionAnalisys(Grid3d* g_3D_ , double bound_obst_, octomap::OcTree* octotree_full_,
                                                    pcl::KdTreeFLANN <pcl::PointXYZ> trav_kdT_, pcl::PointCloud <pcl::PointXYZ>::Ptr trav_pc_)
{
   grid_3D = g_3D_; 
   received_grid = true;
   bound_obst = bound_obst_;
   octotree_full = octotree_full_;
   kdt_trav = trav_kdT_; 
   pc_trav = trav_pc_; 
}

void bisectionCatenary::getStatusCollisionCat(std::vector<double> &dist_obst_cat_, std::vector<int> &pos_cat_in_coll_, std::vector<int> &cat_between_obs_, int &first_coll_, int &last_coll_)
{
    dist_obst_cat_.clear(); pos_cat_in_coll_.clear(); cat_between_obs_.clear();
    dist_obst_cat_ = dist_obst_cat; 
    pos_cat_in_coll_ = pos_cat_in_coll; 
    cat_between_obs_ = cat_between_obs;
    first_coll_ = first_coll; 
    last_coll_ = last_coll;
}
    
void bisectionCatenary::markerPoints(visualization_msgs::MarkerArray _marker, std::vector<geometry_msgs::Point> _vector, ros::Publisher c_m_pub_)
{
    std::string string_marker;
    std::string ns_marker;
            
    ns_marker = "p_b_cat";

    _marker.markers.resize(_vector.size());
            
    for (size_t i = 0; i < _vector.size(); ++i){
        _marker.markers[i].header.frame_id = "/map";
        _marker.markers[i].header.stamp = ros::Time::now();
        _marker.markers[i].ns = ns_marker;
        _marker.markers[i].id = i+1;
        _marker.markers[i].action = visualization_msgs::Marker::ADD;
        _marker.markers[i].type = visualization_msgs::Marker::CUBE;
        _marker.markers[i].lifetime = ros::Duration(0);
        _marker.markers[i].pose.position.x = _vector[i].x; 
        _marker.markers[i].pose.position.y = _vector[i].y; 
        _marker.markers[i].pose.position.z = _vector[i].z;

        _marker.markers[i].pose.orientation.x = 0.0;
        _marker.markers[i].pose.orientation.y = 0.0;
        _marker.markers[i].pose.orientation.z = 0.0;
        _marker.markers[i].pose.orientation.w = 1.0;
        _marker.markers[i].scale.x = 0.01;
        _marker.markers[i].scale.y = 0.01;
        _marker.markers[i].scale.z = 0.01;
        _marker.markers[i].color.a = 1.0;
        _marker.markers[i].color.r = 0.01 ;
        _marker.markers[i].color.g = 0.01;
        _marker.markers[i].color.b = 0.01;
        // printf("INSIDE MARKER _vector.size()=%lu , _vector[%lu][%f %f %f] \n", _vector.size(), i,_vector[i].x,_vector[i].y,_vector[i].z);

    }	
    c_m_pub_.publish(_marker);
}

void bisectionCatenary::clearMarkers(visualization_msgs::MarkerArray _marker, int _s, ros::Publisher c_m_pub_)
{
    _marker.markers.clear();
    _marker.markers.resize(_s);

    for (int i = 0 ; i < _s; i++){
        _marker.markers[i].action = visualization_msgs::Marker::DELETEALL;
    }
    c_m_pub_.publish(_marker);
}