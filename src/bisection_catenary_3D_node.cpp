// #include "marsupial_g2o/bisection3D.hpp"
// #include "iostream"

// struct points_catenary_3D
// {
//     float x_;
//     float y_;
//     float z_;
// };
// points_catenary_3D point_cat3D;

// int main(int argc, char** argv) {
  
//     printf("\n Starting with bisection Method \n");
  
//     bisection B;
//     float Zmin = B.Z2;
//     int pos_lower_point;
//     for(int i=0; i < B.catenary_chain_points_3D.size(); i++)
//     {
//         point_cat3D.x_ = B.catenary_chain_points_3D[i].x_ ;
//         point_cat3D.y_ = B.catenary_chain_points_3D[i].y_;
//         point_cat3D.z_ = B.catenary_chain_points_3D[i].z_;
//         if (Zmin > B.catenary_chain_points_3D[i].z_)
//         {
//             Zmin = B.catenary_chain_points_3D[i].z_;
//             pos_lower_point = i;
//         }
//         printf("Points Catenary X_= %f , Y_= %f , Z = %f\n",
//         B.catenary_chain_points_3D[i].x_,B.catenary_chain_points_3D[i].y_,B.catenary_chain_points_3D[i].z_);
//     }
//     printf("\nFOUND IT!! The lowest point Catenary 3D X_= %f , Y_= %f , Z = %f\n",
//     B.catenary_chain_points_3D[pos_lower_point].x_,B.catenary_chain_points_3D[pos_lower_point].y_,B.catenary_chain_points_3D[pos_lower_point].z_);
  
// // Get parameters from ROS
// //   double x_0, y_0, a_0;
// //   pnh.param("x0", x_0, 0.0);
// //   pnh.param("y0", y_0, 0.0);
// //   pnh.param("a0", a_0, 0.0);
  
// //   double x_g, y_g, a_g;
// //   pnh.param("x_g", x_g, 0.0);
// //   pnh.param("y_g", y_g, 0.0);
// //   pnh.param("a_g", a_g, 0.0);

//   return 0;
// }