#ifndef G2O_VERTEX_CATENARY_LENGTH_H
#define G2O_VERTEX_CATENARY_LENGTH_H


#include "g2o/config.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/hyper_graph_action.h"

#include <Eigen/Core>

namespace g2o
{

/**
  * @class VertexCatenaryLength
  * @brief This class stores and wraps a time difference \f$ \Delta T \f$ into a vertex that can be optimized via g2o
  * @see VertexPointXY
  * @see VertexOrientation
  */
class VertexCatenaryLength : public g2o::BaseVertex<1, double>
{
public:

  /**
    * @brief Default constructor
    * @param fixed if \c true, this vertex is considered fixed during optimization [default: \c false]
    */  
  VertexCatenaryLength(bool fixed = false)
  {
    setToOriginImpl();
    setFixed(fixed);
  }
  
  /**
    * @brief Destructs the VertexCatenaryLength
    */ 
  ~VertexCatenaryLength()
  {}

  /**
    * @brief Access the catenary length of the vertex
    * @see estimate
    * @return reference to cfactor
    */ 
  double& cfactor() {return _estimate;}
  
  /**
    * @brief Access the catenary length value of the vertex (read-only)
    * @see estimate
    * @return const reference to cfactor
    */ 
  const double& cfactor() const {return _estimate;}
  
  /**
    * @brief Set the underlying Length estimate to default.
    */ 
  virtual void setToOriginImpl()
  {
    _estimate = 0.1;
  }

  /**
    * @brief Define the update increment \f$ \Length{k+1} = \Length_k + update \f$.
    * A simple addition implements what we want.
    * @param update increment that should be added to the previous esimate
    */ 
  virtual void oplusImpl(const double* update)
  {
      _estimate += *update;
  }

  /**
    * @brief Read an estimate of \f$ \Length \f$ from an input stream
    * @param is input stream
    * @return always \c true
    */ 
  virtual bool read(std::istream& is)
  {
    is >> _estimate;
    return true;
  }

  /**
    * @brief Write the estimate \f$ \Length \f$ to an output stream
    * @param os output stream
    * @return \c true if the export was successful, otherwise \c false
    */ 
  virtual bool write(std::ostream& os) const
  {
    os << estimate();
    return os.good();
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}

#endif
