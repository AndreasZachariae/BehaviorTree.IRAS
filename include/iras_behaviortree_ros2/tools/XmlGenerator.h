/** *******************************************************
 * IRAS - University of Applied Sciences Karlsruhe
 * Module : iras_behaviortree_ros2
 * Purpose : Tool for generating the XML Groot palette
 *
 * @author Andreas Zachariae
 * @since 2.0.0 (2023.12.13)
 *********************************************************/
#pragma once

#include <tinyxml2.h>
#include <tuple>

#include <behaviortree_cpp_v3/bt_factory.h>
#include <iras_behaviortree_ros2/default.h>

class XmlGenerator
{
public:
    XmlGenerator(BT::BehaviorTreeFactory *factory) : factory_(factory){};

    void generate_xml_palette(const std::string &filename)
    {
        tinyxml2::XMLDocument doc;
        tinyxml2::XMLElement *root = doc.NewElement("root");
        doc.InsertFirstChild(root);

        tinyxml2::XMLElement *treeNodesModel = doc.NewElement("TreeNodesModel");
        root->InsertEndChild(treeNodesModel);

        const std::unordered_map<std::string, BT::TreeNodeManifest> &manifest = factory_->manifests();

        for (const auto &node : manifest)
        {
            if (standard_nodes.find(node.first) != standard_nodes.end())
            {
                continue;
            }

            tinyxml2::XMLElement *xml_node = doc.NewElement(to_string(node.second.type));
            xml_node->SetAttribute("ID", node.first.c_str());
            treeNodesModel->InsertEndChild(xml_node);

            for (const auto &port : node.second.ports)
            {
                tinyxml2::XMLElement *xml_port = doc.NewElement(to_string(port.second.direction()));
                xml_port->SetAttribute("name", port.first.c_str());
                xml_port->SetAttribute("type", to_string(*port.second.type()));
                xml_node->InsertEndChild(xml_port);
            }
        }

        doc.SaveFile(filename.c_str());
    }

private:
    BT::BehaviorTreeFactory *factory_;

    const char *to_string(const BT::NodeType &type)
    {
        switch (type)
        {
        case BT::NodeType::ACTION:
            return "Action";
        case BT::NodeType::CONDITION:
            return "Condition";
        case BT::NodeType::DECORATOR:
            return "Decorator";
        case BT::NodeType::CONTROL:
            return "Control";
        case BT::NodeType::SUBTREE:
            return "SubTree";
        default:
            return "Undefined";
        }
    }

    const char *to_string(const BT::PortDirection &direction)
    {
        switch (direction)
        {
        case BT::PortDirection::INPUT:
            return "input_port";
        case BT::PortDirection::OUTPUT:
            return "output_port";
        case BT::PortDirection::INOUT:
            return "inout_port";
        }
    }

    const char *to_string(const std::type_info &type)
    {
        if (typeid(float) == type)
        {
            return "float";
        }
        else if (typeid(int) == type)
        {
            return "int";
        }
        else if (typeid(bool) == type)
        {
            return "bool";
        }
        else if (typeid(std::string) == type)
        {
            return "std::string";
        }
        else
        {
            return "undefined";
        }
    }

    const std::unordered_set<std::string> standard_nodes = {
        "Sequence",
        "Fallback",
        "Parallel",
        "ReactiveFallback",
        "ReactiveSequence",
        "ManualSelector",
        "Repeat",
        "WhileDoElse",
        "IfThenElse",
        "AlwaysFailure",
        "AlwaysSuccess",
        "ForceFailure",
        "ForceSuccess",
        "KeepRunningUntilFailure",
        "RetryUntilSuccessful",
        "SetBlackboard",
        "SubTree",
        "SubTreePlus",
        "Inverter",
        "Switch2",
        "Switch3",
        "Switch4",
        "Switch5",
        "Switch6",
        "BlackboardCheckInt",
        "BlackboardCheckDouble",
        "BlackboardCheckBool",
        "BlackboardCheckString",
        "Delay",
        "SequenceStar",
        "Timeout",
    };
};
