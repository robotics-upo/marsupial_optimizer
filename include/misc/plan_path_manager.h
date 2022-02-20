#ifndef __PATH_MANAGE_H__
#define __PATH_MANAGE_H__

#include <nav_msgs/Path.h>
#include <QTableWidget>
#include <QMessageBox>
#include <tinyxml.h>
#include <memory>
#include <yaml-cpp/yaml.h>
#include <tf/tf.h>

enum WaypointType {
    INSPECTION, INTERMEDIATE, SHELTER, RAMP, UNKNOWN
};

class PlanPath:public QTableWidget 
{
    Q_OBJECT 

    public:
        PlanPath(QWidget* parent = (QWidget *)nullptr, const std::string &reference_mission_filename = "reference_mission.xml");

        //! @brief Translates the info to 
        nav_msgs::Path toNavigationPath() const;

        public slots:
        void updatePlan(const nav_msgs::Path &plan);

        //! @brief Adds a waypoint at the end of the list
        void addWaypoint(const QString &s = QString(""), double x = 0.0, double y = 0.0, double yaw = 0.0, int type = 0);

        void newWaypoint();

        void clearMission();

        void deleteWaypoint();

        void moveWaypointUp();
        void moveWaypointDown();

        void interchangeWaypoints(int i, int j);

        //! @brief 
        void handleValueChanged(int i, int j);

        void handleComboChanged (int i);

        //! @brief loads a mission
        //! @note Put only the name of the file without extension. It would search for a XML and YAML files
        bool loadMission(const QString &filename);

        bool exportMission(const QString &filename);

    signals:
        void planChanged(const nav_msgs::Path &p);
        
    public:
        std::string frame_id;

    protected:
        bool update_on_change = true;
        std::unique_ptr<TiXmlDocument> reference_doc;
        TiXmlNode *ramp, *inspection, *intermediate, *shelter, *tree_nodes_model;

        bool getReferenceMission(const std::string &name);

        void addWaypointByNode(const TiXmlNode *n, const YAML::Node &config);

        void getWaypointFromYAML(const YAML::Node &yaml_node, const std::string &name, double &x, double &y, double &yaw);

        int addYAMLPoint(YAML::Node &y, int i, std::string &name);
    
    public:

        static double quaternion2yaw(double x, double y, double z, double w);
        static tf::Quaternion yaw2quaternion(double yaw);
};

#endif