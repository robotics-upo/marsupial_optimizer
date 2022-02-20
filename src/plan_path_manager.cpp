#include "plan_path_manager.h"
#include <QComboBox>
#include <QPushButton>
#include <tf/LinearMath/Matrix3x3.h>
#include <fstream>

using namespace std;

PlanPath::PlanPath(QWidget *parent, const std::string &reference_mission_filename):QTableWidget(parent),frame_id("map")
{
    // Write the header
    QTableWidgetItem *newItem = new QTableWidgetItem("Name");
    setRowCount(1);
    setColumnCount(5); // Number of columns: Name, x, y, yaw, type
    setItem(0, 0, newItem);
    newItem = new QTableWidgetItem("X Coord (m)");
    setItem(0, 1, newItem);
    newItem = new QTableWidgetItem("Y Coord (m)");
    setItem(0, 2, newItem);
    newItem = new QTableWidgetItem("Yaw Coord (m)");
    setItem(0, 3, newItem);
    newItem = new QTableWidgetItem("Type");
    setItem(0, 4, newItem);

    setMaximumHeight(300);

    getReferenceMission(reference_mission_filename);

    // Set a little wider all columns
    for (int i=1; i < columnCount(); i++) {
        setColumnWidth(i, 150);
    }
        
    // The first a little bit more
    setColumnWidth(0, 400);
}

void PlanPath::updatePlan(const nav_msgs::Path &plan) {
    setRowCount(1); // The first is the Header!
    ROS_INFO("Updating plan");
    update_on_change = false;
    frame_id = plan.header.frame_id;
    for (size_t i = 0; i < plan.poses.size(); i++) {
        const geometry_msgs::PoseStamped &p = plan.poses[i];

        double yaw = quaternion2yaw(p.pose.orientation.x, 
                                    p.pose.orientation.y,
                                    p.pose.orientation.z, 
                                    p.pose.orientation.w);
            
        addWaypoint(QString::fromStdString(p.header.frame_id), p.pose.position.x, p.pose.position.y, yaw, p.header.seq);
    }
    // emit planChanged(toNavigationPath()); // TODO: necessary?
    update_on_change = true;
}

void PlanPath::clearMission() {
    setRowCount(1);
    emit planChanged(toNavigationPath());
}

void PlanPath::addWaypoint(const QString &s, double x, double y, double yaw, int type) {
    ROS_INFO("Adding waypoint");
    int i = this->rowCount();
    setRowCount(i + 1);
    QTableWidgetItem *newItem = new QTableWidgetItem(s);
    setItem(i, 0, newItem);
    newItem = new QTableWidgetItem(QString::number(x));
    setItem(i, 1, newItem);
    newItem = new QTableWidgetItem(QString::number(y));
    setItem(i, 2, newItem);
    newItem = new QTableWidgetItem(QString::number(yaw));
    setItem(i, 3, newItem);

    QComboBox *typeCombo = new QComboBox;
    typeCombo->addItem("Inspection");
    typeCombo->addItem("Intermediate");
    typeCombo->addItem("Shelter");
    typeCombo->addItem("Ramp");

    connect(typeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &PlanPath::handleComboChanged);

    if (INSPECTION <=type && type < UNKNOWN) {
        typeCombo->setCurrentIndex(type);
    } else {
        typeCombo->setCurrentIndex(INSPECTION);
    }
    setCellWidget(i , 4, typeCombo);

    ROS_INFO("Added waypoint");
}

void PlanPath::moveWaypointUp() {
    if (currentRow() > 1 && currentRow() < rowCount()) 
        interchangeWaypoints(currentRow() - 2, currentRow() - 1);
    emit planChanged(toNavigationPath()); 
}

void PlanPath::moveWaypointDown() {
    if (currentRow() > 0 && currentRow() < rowCount() - 1) 
        interchangeWaypoints(currentRow() - 1, currentRow());

    emit planChanged(toNavigationPath()); 
}

void PlanPath::interchangeWaypoints(int i, int j) 
{
    auto plan = toNavigationPath();
    nav_msgs::Path new_plan;
    new_plan.header = plan.header;

    int cont = 0;
    for (auto wp:plan.poses) {
        if (cont == i) {
            new_plan.poses.push_back(plan.poses.at(j));
        } else if (cont == j) {
            new_plan.poses.push_back(plan.poses.at(i));
        } else {
            new_plan.poses.push_back(wp);
        }
        cont ++;
    }

    updatePlan(new_plan);
}

nav_msgs::Path PlanPath::toNavigationPath() const {
    nav_msgs::Path p;
    static int seq = 0;
    p.header.frame_id = frame_id;
    p.header.seq = seq++;
    auto t = ros::Time::now();
    p.header.stamp = t;
        
    for (int i = 1; i < rowCount(); i++) {

        // Check for NULLs:
        bool null = false;
        for (int j = 0; j < columnCount() - 1; j++) {
            if (item(i,j) == nullptr)
                null = true;
        }
        if (null || cellWidget(i, 4) == nullptr) {
            ROS_INFO("Skipping waypoint %d", i);
            continue;
        }

        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = item(i, 0)->text().toStdString();
        pose.header.stamp = t;
        QComboBox *curr_combo = dynamic_cast<QComboBox *>(cellWidget(i, 4));
        if (curr_combo != nullptr)
            pose.header.seq = dynamic_cast<QComboBox *>(cellWidget(i, 4))->currentIndex();
        else
            pose.header.seq = 0;

        pose.pose.position.x = item(i, 1)->text().toFloat();
        pose.pose.position.y = item(i, 2)->text().toFloat();
        double yaw = item(i, 3)->text().toFloat();

        tf::Quaternion q = yaw2quaternion(yaw);
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();

        p.poses.push_back(pose);
    }
    return p;
}

void PlanPath::handleValueChanged(int i, int j) {
    if (update_on_change) {
        emit planChanged(toNavigationPath());
    }
}

void PlanPath::handleComboChanged(int i) {
    if (update_on_change) {
        emit planChanged(toNavigationPath());
    }
}

void PlanPath::newWaypoint() 
{
    ROS_INFO("New wp");
    QString s("inspection_point_");
    s.append(QString::number(rowCount()));
    addWaypoint(s);
    ROS_INFO("New wp created");
    emit planChanged(toNavigationPath());
    ROS_INFO("Send wp list");
}

bool PlanPath::loadMission(const QString &filename) 
{
    setRowCount(1); // Clear mission
    update_on_change = false;

    QString xml_file(filename); 
    ROS_INFO("XML File name: %s", xml_file.toStdString().c_str());
    xml_file.replace("file://", ""); // TODO: necessary? HANDLE SFTP

    QString yaml_file(xml_file);
    YAML::Node config;
    yaml_file.replace(QString(".xml"), QString(".yaml"));
    try {
        config = YAML::LoadFile(yaml_file.toStdString());
    } catch (exception &e) {
        ROS_INFO("Could not load YAML File. Content %s", e.what());
        return false;
    }

    ROS_INFO("Loading %s tree file.", xml_file.toStdString().c_str());

    TiXmlDocument doc(xml_file.toStdString().c_str());
    if (!doc.LoadFile() ) {
        return false;
    }
    TiXmlElement *root = doc.RootElement();

    if (!root) {
      cerr << "Could not get the xml behavior tree file.\n";
      return false;
    }
    string main_tree_name(root->ToElement()->Attribute("main_tree_to_execute"));
    ROS_INFO("Load tree: main tree to execute: %s", main_tree_name.c_str());
    // Iterate over the children and copy the argument data
    TiXmlNode *it = root->FirstChild();
        
    while (it) {
        if (it->ValueStr() == "BehaviorTree" && it->ToElement()) {
            string name(it->ToElement()->Attribute("ID"));
            cout << "Tree name: " << name << endl;
            if (name == main_tree_name) {
                // We have reached the main tree, get the subtrees
                TiXmlNode *sequence_node = it->FirstChild(); // Get into the sequence node
                if (!sequence_node)
                    return false;
                TiXmlNode *it2 = sequence_node->FirstChild();

                while (it2) {
                    cout << "Main tree node: " << it2->ValueStr() << endl;
                    if (it2->ValueStr() == "SubTree" && it2->ToElement()) {
                        addWaypointByNode(it2, config);
                    }
                    // TODO: Get the coords from YAML and the structure from XML
                    it2 = sequence_node->IterateChildren(it2);
                }
                break; // We are done --> exit
            }
        }
        it = root->IterateChildren(it);
    }
    emit planChanged(toNavigationPath());
    update_on_change = true;
    return true;
}

bool PlanPath::getReferenceMission(const std::string &name) 
{
    ramp = tree_nodes_model = shelter = intermediate = inspection = NULL;
    ROS_INFO("Loading reference mission: %s", name.c_str());
    reference_doc.reset(new TiXmlDocument(name.c_str()));
    if (!reference_doc->LoadFile()) {
      ROS_ERROR( "Could not open the behavior tree file.\n");
      return false;  
    }
    TiXmlElement *root = reference_doc->RootElement();

    if (!root) {
      cerr << "Could not get the reference xml behavior tree file.\n";
      return false;
    }

    // Get the Ramp, Inspection, Intermediate, Shelter subtrees 
    TiXmlNode *it = root->FirstChild();
        
    while (it) {
        if (it->ValueStr() == "BehaviorTree" && it->ToElement()) {
            string name(it->ToElement()->Attribute("ID"));
            cout << "Tree name: " << name << endl;
            if (name == "GoToShelter") {
                ROS_INFO("Detected shelter node");
                shelter = it;
            } else if (name == "Inspection") {
                ROS_INFO("Detected inspection node");
                inspection = it;
            } else if (name == "Intermediate") {
                ROS_INFO("Detected intermediate node");
                intermediate = it;
            } else if (name == "GoThroughRamp") {
                ROS_INFO("Detected ramp node");
                ramp = it;
            }
        }
        if (it->ValueStr() == "TreeNodesModel" && it->ToElement()) {
            tree_nodes_model = it;
        }
        it = root->IterateChildren(it);
    }
    return true;
}

bool PlanPath::exportMission(const QString &filename) 
{
    QString xml_file(filename); 
    if(!xml_file.endsWith(".xml"))
        xml_file.append(".xml");

    xml_file.replace("file://", ""); // TODO: necessary? HANDLE SFTP
    ROS_ERROR("XML File: %s", xml_file.toStdString().c_str());
    QString yaml_file(xml_file);
    YAML::Node yaml;
    yaml_file.replace(QString(".xml"), QString(".yaml"));
    ROS_ERROR("YAML File: %s", yaml_file.toStdString().c_str());

    // Create document
    TiXmlDocument doc;
    TiXmlDeclaration decl( "1.0", "", "" );  
    doc.InsertEndChild( decl );  
        
    // Create root launch node
    TiXmlElement root_element("root");
    root_element.SetAttribute("main_tree_to_execute", "BehaviorTree");

    TiXmlElement main_tree("BehaviorTree");
    main_tree.SetAttribute("ID", "BehaviorTree");
        
    TiXmlElement seq("Sequence");
        
    bool shelter_is_last = false;

    // Go through all waypoints
    for (int i = 0; i < rowCount(); i++) {
        TiXmlElement *sub_tree = new TiXmlElement("SubTree");
        std::string wp_name;
        int type = addYAMLPoint(yaml, i, wp_name);
        if (type < 0)
            continue;

        if (type >= UNKNOWN) {
            type = INTERMEDIATE;
            shelter_is_last = false;
        }

        ostringstream msg;
        if (type == RAMP) {
            sub_tree->SetAttribute("ramp_1", wp_name);
            sub_tree->SetAttribute("ID", "GoThroughRamp");
            msg << "Went through ramp. First point: "<< wp_name;
            // Get the next waypoint
            if (rowCount() <= i + 1) {
                ROS_INFO("Skipping ramp waypoint %d", i);
                continue; // Not enough ramp points
            }
            i++;
            type = addYAMLPoint(yaml, i, wp_name);
            msg << "  Second point: " << wp_name;
                
            if (type != RAMP) {
                 ROS_INFO("Warning: ramp isolated point. Considering the next as the end of the ramp");
            }
            sub_tree->SetAttribute("ramp_2", wp_name);
            shelter_is_last = false;
        } else if (type == INSPECTION) {
            msg << "Arrived to inspection point " << wp_name;
            sub_tree->SetAttribute("ID", "Inspection");
            sub_tree->SetAttribute("inspection_point", wp_name);
            sub_tree->SetAttribute("inspect_time", 10);
            shelter_is_last = false;
        } else if (type == INTERMEDIATE) {
            sub_tree->SetAttribute("ID", "Intermediate");
            msg << "Arrived to intermediate point " << wp_name;
            sub_tree->SetAttribute("intermediate_waypoint", wp_name);
            shelter_is_last = false;
        } else if (type == SHELTER) {
            sub_tree->SetAttribute("ID", "GoToShelter");
            msg << "Reached Shelter position.";
            shelter_is_last = true;
        }
        ostringstream os;
        os << "state_" << i;
        sub_tree->SetAttribute("final_state", os.str());
        sub_tree->SetAttribute("message", msg.str());

        seq.LinkEndChild(sub_tree);
    }

    main_tree.InsertEndChild(seq);
    root_element.InsertEndChild(main_tree);
    // Add the remaining subtrees
    if (ramp)
        root_element.InsertEndChild(*ramp);
    if (inspection)
        root_element.InsertEndChild(*inspection);
    if (intermediate)
        root_element.InsertEndChild(*intermediate);
    if (shelter)
        root_element.InsertEndChild(*shelter);
    if (tree_nodes_model)
        root_element.InsertEndChild(*tree_nodes_model);
    if(!shelter_is_last)
        QMessageBox::warning(this, tr("Warning"), tr("Shelter position is not the last one"));
            
    // Save YAML
    try {
        ofstream ofs( yaml_file.toStdString());
        ofs << yaml;
    } catch (exception &e) {
        ROS_ERROR("Exception saving yaml file: %s", e.what());
        return false;
    }

    // Save XML
    doc.InsertEndChild(root_element);
    doc.SaveFile(xml_file.toStdString());

    return true;
}

void PlanPath::addWaypointByNode(const TiXmlNode *n, const YAML::Node &yaml_node) {
    try {
        string name(n->ToElement()->Attribute("ID"));
        // Get the type
        string waypoint_name = "shelter";
        int type = INTERMEDIATE; 
        double x = 0.0;
        double y = 0.0;
        double yaw = 0.0;
            
        if (name == "GoToShelter") {
            ROS_INFO("Adding shelter node. ");
            type = SHELTER;
        } else if (name == "Inspection") {
            ROS_INFO("Detected inspection node");
            waypoint_name = n->ToElement()->Attribute("inspection_point");
            type = INSPECTION;
        } else if (name == "Intermediate") {
            ROS_INFO("Detected intermediate node");
            waypoint_name = n->ToElement()->Attribute("intermediate_waypoint");
            type = INTERMEDIATE;
        } else if (name == "GoThroughRamp") {
            ROS_INFO("Detected ramp node");
            // Here we have to add two different waypoints (one for entering the ramp other at the exit)
            type = RAMP;
            // getCoordinates();
            waypoint_name = n->ToElement()->Attribute("ramp_1");
            getWaypointFromYAML(yaml_node, waypoint_name, x, y ,yaw);
            addWaypoint(QString::fromStdString(waypoint_name), x, y, yaw, type);
            waypoint_name = n->ToElement()->Attribute("ramp_2");
        }
        getWaypointFromYAML(yaml_node, waypoint_name, x, y ,yaw);
        addWaypoint(QString::fromStdString(waypoint_name), x, y, yaw, type);
    } catch(exception &ex){
        ROS_WARN("Could not add waypoint");
    }
}

double PlanPath::quaternion2yaw(double x, double y, double z, double w) {
    double roll, pitch, yaw;
    tf::Quaternion q(x, y, z, w);
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
    
    return yaw;
}

tf::Quaternion PlanPath::yaw2quaternion(double yaw) {
    tf::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);

    return q;
}

void PlanPath::getWaypointFromYAML(const YAML::Node &yaml_node, const std::string &name, double &x, double &y, double &yaw) 
{
    x = y = yaw = 0;
    try{
        x = yaml_node[name]["pose"]["x"].as<double>();
        y = yaml_node[name]["pose"]["y"].as<double>();
        double qx, qy, qz, qw;
        qx = yaml_node[name]["orientation"]["x"].as<double>();
        qy = yaml_node[name]["orientation"]["y"].as<double>();
        qz = yaml_node[name]["orientation"]["z"].as<double>();
        qw = yaml_node[name]["orientation"]["w"].as<double>();
        yaw = quaternion2yaw(qx, qy, qz, qw);
    }catch(YAML::InvalidNode &ex){
        ROS_WARN("Invalid node %s, ex: %s", name.c_str(), ex.what());
    }
}

int PlanPath::addYAMLPoint(YAML::Node &y, int i, std::string &name) 
{
    // Check for NULLs:
    bool null = false;
    for (int j = 0; j < columnCount() - 1; j++) {
        if (item(i,j) == nullptr)
            null = true;
    }
    if (null || cellWidget(i, 4) == nullptr) {
        ROS_INFO("Skipping waypoint %d", i);
        return -1;
    }

    int type = dynamic_cast<QComboBox *>(cellWidget(i, 4))->currentIndex();

    QString waypoint_name("shelter");
    if (type != SHELTER) {
        waypoint_name = item(i, 0)->text();
    }
    name = waypoint_name.toStdString();

    double x = item(i, 1)->text().toFloat();
    double y_ = item(i, 2)->text().toFloat();
    double yaw = item(i, 3)->text().toFloat();
    tf::Quaternion q = yaw2quaternion(yaw); 

    y[name]["pose"]["x"] = x;
    y[name]["pose"]["y"] = y_;
    y[name]["pose"]["z"] = 0.0;
    y[name]["orientation"]["x"] = q.x();
    y[name]["orientation"]["y"] = q.y();
    y[name]["orientation"]["z"] = q.z();
    y[name]["orientation"]["w"] = q.w();

    return type;
}

void PlanPath::deleteWaypoint() 
{
    std::set<int> row_set;
    for ( auto x:this->selectedItems() ) {
        row_set.insert(x->row());
    }
    while (!row_set.empty()) {
        removeRow(*row_set.crend());
        row_set.erase(*row_set.crend());
    }
    emit planChanged(toNavigationPath());
}