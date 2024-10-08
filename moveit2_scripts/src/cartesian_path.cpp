#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <queue>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");
static const double jump_threshold = 0.0;
static const double eef_step = 0.01;

enum class State {
    PICK,
    APPROACH,
    RETREAT,
    PLACE,
    END
};

std::queue<geometry_msgs::msg::Pose> get_objects_position();

void state_machine(moveit::planning_interface::MoveGroupInterface &move_group_arm,
                   const std::shared_ptr<rclcpp::Client<std_srvs::srv::SetBool>> client);

void add_collision_scene(moveit::planning_interface::MoveGroupInterface &move_group_arm);

void set_path_constraint(moveit::planning_interface::MoveGroupInterface &move_group_arm);

bool pick(moveit::planning_interface::MoveGroupInterface &move_group_arm, 
          geometry_msgs::msg::Pose target_pose);

bool approach(moveit::planning_interface::MoveGroupInterface &move_group_arm,
              geometry_msgs::msg::Pose target_pose);

bool retreat(moveit::planning_interface::MoveGroupInterface &move_group_arm,
             geometry_msgs::msg::Pose target_pose);

bool place(moveit::planning_interface::MoveGroupInterface &move_group_arm,
           geometry_msgs::msg::Pose target_pose);

bool call_vaccum_gripping_service(const std::shared_ptr<rclcpp::Client<std_srvs::srv::SetBool>> client, 
                                  bool data); 

int main(int argc, char **argv) {

    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node =
        rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);

    auto client = move_group_node->create_client<std_srvs::srv::SetBool>("/demo/switch_demo");

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_group_node);
    std::thread([&executor]() { executor.spin(); }).detach();

    static const std::string PLANNING_GROUP_ARM = "ur_manipulator";

    moveit::planning_interface::MoveGroupInterface move_group_arm(
        move_group_node, PLANNING_GROUP_ARM);


    const moveit::core::JointModelGroup *joint_model_group_arm =
        move_group_arm.getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);

    move_group_arm.setPlannerId("RRTstar");
    move_group_arm.setPlanningTime(10.0);

    /*Get Current State*/
    moveit::core::RobotStatePtr current_state_arm =
        move_group_arm.getCurrentState(10);

    std::vector<double> joint_group_positions_arm;
    current_state_arm->copyJointGroupPositions(joint_model_group_arm,
                                                joint_group_positions_arm);

    move_group_arm.setStartStateToCurrentState();

    /* Add collision object*/
    add_collision_scene(move_group_arm);

    // set_path_constraint(move_group_arm);

    /* Set velocity and acceleration limits*/
    move_group_arm.setMaxVelocityScalingFactor(0.4);
    move_group_arm.setMaxAccelerationScalingFactor(0.2);

    // move_group_arm.setWorkspace(-0.3, -0.2, -0.05, 0.9, 1.0, 1.05);

    /* Start the FSM*/
    state_machine(move_group_arm, client);


    rclcpp::shutdown();
    return 0;
}


std::queue<geometry_msgs::msg::Pose> get_positions()
{
    std::queue<geometry_msgs::msg::Pose> positions;

    geometry_msgs::msg::Pose target_pose;


    target_pose.orientation.x = -1.0;
    target_pose.orientation.y = 0.00;
    target_pose.orientation.z = 0.00;
    target_pose.orientation.w = 0.00;
    target_pose.position.z = 0.260;

    // pose1
    target_pose.position.x = 0.343;
    target_pose.position.y = 0.278;

    positions.push(target_pose);

    // pose2
    target_pose.position.x = 0.25;
    target_pose.position.y = -0.25;

    positions.push(target_pose);

    // pose3
    target_pose.position.x = 0.343;
    target_pose.position.y = 0.422;

    positions.push(target_pose);

    // pose4
    target_pose.position.x = 0.35;
    target_pose.position.y = -0.25;

    positions.push(target_pose);

    // pose5
    target_pose.position.x = 0.21;
    target_pose.position.y = 0.422;

    positions.push(target_pose);

    // pose6
    target_pose.position.x = 0.25;
    target_pose.position.y = -0.35;

    positions.push(target_pose);

    // pose7
    target_pose.position.x = 0.21;
    target_pose.position.y = 0.278;

    positions.push(target_pose);

    // pose8
    target_pose.position.x = 0.35;
    target_pose.position.y = -0.35;

    positions.push(target_pose);

    return positions;
}


void state_machine(moveit::planning_interface::MoveGroupInterface &move_group_arm,
                   const std::shared_ptr<rclcpp::Client<std_srvs::srv::SetBool>> client) 
{
    std::queue<geometry_msgs::msg::Pose> robot_desired_pos = get_positions();

    State currentState = State::PICK;
    State previousState = State::PICK;

    geometry_msgs::msg::Pose target_pose;

    while (true) {
        switch (currentState) {
                  
        case State::PICK:
            if (robot_desired_pos.empty()) {
                currentState = State::END;
                break;
            }
            target_pose = robot_desired_pos.front(); 
            if (!pick(move_group_arm, target_pose)) 
            {
                currentState = State::END;
            } 
            else 
            {
                currentState = State::APPROACH;
                previousState = State::PICK;
            }
            break;

        case State::APPROACH:
            if (!approach(move_group_arm, target_pose)) 
            {
                currentState = State::END;
            } 
            else 
            {
                bool result = false;
                if (previousState == State::PICK)
                {
                    result = call_vaccum_gripping_service(client, true);
                }
                else if (previousState == State::PLACE)
                {
                    result = call_vaccum_gripping_service(client, false);
                }

                if (result) 
                {
                    currentState = State::RETREAT;
                } 
                else 
                {
                    currentState = State::END;
                }
            }
            break;

        case State::RETREAT:
            if (!retreat(move_group_arm, target_pose)) 
            {
                currentState = State::END;
            } 
            else 
            {
                if (robot_desired_pos.empty())
                {
                    currentState = State::END;
                    break;
                }
                else
                {
                    robot_desired_pos.pop();
                    if (previousState == State::PICK)
                    {
                        currentState = State::PLACE;
                    }
                    else if (previousState == State::PLACE)
                    {
                        currentState = State::PICK;
                    }
                }
            }
            break;

        case State::PLACE:
            
            if (robot_desired_pos.empty()) {
                currentState = State::END;
                break;
            }
            target_pose = robot_desired_pos.front();

            if (!place(move_group_arm, target_pose)) 
            {
                currentState = State::END;
            } 
            else 
            {
                currentState = State::APPROACH;
                previousState = State::PLACE;
            }
            break;

        case State::END:
            return;
                
        default:
            std::cerr << "Error: Unknown state encountered." << std::endl;
            return;
        }
    }
}

bool pick(moveit::planning_interface::MoveGroupInterface &move_group_arm, 
          geometry_msgs::msg::Pose target_pose)
{
    moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;

    RCLCPP_INFO(LOGGER, "PICK: x = %f, y = %f, z = %f",  
            target_pose.position.x, target_pose.position.y, target_pose.position.z);

    move_group_arm.setPoseTarget(target_pose);

    bool success_arm = (move_group_arm.plan(my_plan_arm) ==
                    moveit::core::MoveItErrorCode::SUCCESS);

    move_group_arm.execute(my_plan_arm);

    return success_arm;
}

bool approach(moveit::planning_interface::MoveGroupInterface &move_group_arm, 
              geometry_msgs::msg::Pose target_pose)
{
    std::vector<geometry_msgs::msg::Pose> approach_waypoints;
    target_pose.position.z -= 0.04;
    approach_waypoints.push_back(target_pose);

    target_pose.position.z -= 0.04;
    approach_waypoints.push_back(target_pose);

    target_pose.position.z -= 0.04;
    approach_waypoints.push_back(target_pose);

    target_pose.position.z -= 0.04;
    approach_waypoints.push_back(target_pose);

    // target_pose.position.z -= 0.04;
    // approach_waypoints.push_back(target_pose);

    moveit_msgs::msg::RobotTrajectory trajectory_approach;

    double fraction = move_group_arm.computeCartesianPath(
        approach_waypoints, eef_step, jump_threshold, trajectory_approach);

    move_group_arm.execute(trajectory_approach);

    return true;
}

bool retreat(moveit::planning_interface::MoveGroupInterface &move_group_arm,
             geometry_msgs::msg::Pose target_pose)
{
    std::vector<geometry_msgs::msg::Pose> retreat_waypoints;
    target_pose.position.z += 0.04;
    retreat_waypoints.push_back(target_pose);

    moveit_msgs::msg::RobotTrajectory trajectory_retreat;

    double fraction = move_group_arm.computeCartesianPath(
        retreat_waypoints, eef_step, jump_threshold, trajectory_retreat);

    move_group_arm.execute(trajectory_retreat);

    return true;
}

bool place(moveit::planning_interface::MoveGroupInterface &move_group_arm,
           geometry_msgs::msg::Pose target_pose)
{

    moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;

    RCLCPP_INFO(LOGGER, "PLACE: x = %f, y = %f, z = %f",  
            target_pose.position.x, target_pose.position.y, target_pose.position.z);


    move_group_arm.setPoseTarget(target_pose);

    bool success_arm = (move_group_arm.plan(my_plan_arm) ==
                    moveit::core::MoveItErrorCode::SUCCESS);

    move_group_arm.execute(my_plan_arm);

    return success_arm;

}

void add_collision_scene(moveit::planning_interface::MoveGroupInterface &move_group_arm)
{
    auto const collision_object_1 = [frame_id =
                                    move_group_arm.getPlanningFrame()] {
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = frame_id;
        collision_object.id = "box1";
        shape_msgs::msg::SolidPrimitive primitive;

        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[primitive.BOX_X] = 2.0;
        primitive.dimensions[primitive.BOX_Y] = 2.0;
        primitive.dimensions[primitive.BOX_Z] = 2.0;

        geometry_msgs::msg::Pose box_pose;
        box_pose.orientation.w = 1.0;
        box_pose.position.x = 0.0;
        box_pose.position.y = 0.0;
        box_pose.position.z = -1.02;

        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(box_pose);
        collision_object.operation = collision_object.ADD;

        return collision_object;
    }();

    auto const collision_object_2 = [frame_id =
                                    move_group_arm.getPlanningFrame()] {
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = frame_id;
        collision_object.id = "box2";
        shape_msgs::msg::SolidPrimitive primitive;

        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[primitive.BOX_X] = 0.8;
        primitive.dimensions[primitive.BOX_Y] = 0.8;
        primitive.dimensions[primitive.BOX_Z] = 0.8;

        geometry_msgs::msg::Pose box_pose;
        box_pose.orientation.w = 1.0;
        box_pose.position.x = -0.5;
        box_pose.position.y = -0.5;
        box_pose.position.z = 0.4;

        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(box_pose);
        collision_object.operation = collision_object.ADD;

        return collision_object;
    }();

    // Add both collision objects to the scene
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    planning_scene_interface.applyCollisionObjects({collision_object_1});
}

void set_path_constraint(moveit::planning_interface::MoveGroupInterface &move_group_arm)
{
    moveit_msgs::msg::Constraints constraints;

    // Orientation Constraint
    moveit_msgs::msg::OrientationConstraint ocm;
    ocm.link_name = "wrist_3_link";  // End effector link name
    ocm.header.frame_id = "base_link";
    ocm.orientation.w = 1.0;  // Desired orientation (e.g., keep upright)
    ocm.absolute_x_axis_tolerance = 0.1;
    ocm.absolute_y_axis_tolerance = 0.1;
    ocm.absolute_z_axis_tolerance = 0.1;
    ocm.weight = 1.0;

    // Position Constraint
    moveit_msgs::msg::PositionConstraint pcm;
    pcm.link_name = "wrist_3_link";  // End effector link name
    pcm.header.frame_id = "base_link";
    pcm.constraint_region.primitives.resize(1);
    pcm.constraint_region.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    pcm.constraint_region.primitives[0].dimensions = {2.0, 2.0, 2.0};  // Define the allowed region
    pcm.constraint_region.primitive_poses.resize(1);
    pcm.constraint_region.primitive_poses[0].position.z = 1.0;  // Keep the end effector above a certain height
    pcm.weight = 1.0;

    constraints.orientation_constraints.push_back(ocm);
    constraints.position_constraints.push_back(pcm);

    move_group_arm.setPathConstraints(constraints);
}

bool call_vaccum_gripping_service(const std::shared_ptr<rclcpp::Client<std_srvs::srv::SetBool>> client, 
                                  bool data) 
{
    // Wait for the service to be available
    if (!client->wait_for_service(std::chrono::seconds(10))) {
        RCLCPP_ERROR(LOGGER, "Service /demo/switch_demo not available");
        return false;
    }

    // Create the request
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = data;

    // Call the service
    auto future = client->async_send_request(request);

    // Wait for the result
    if (rclcpp::spin_until_future_complete(rclcpp::Node::make_shared("temp_node"), future) == rclcpp::FutureReturnCode::SUCCESS) 
    {
        auto response = future.get();
        if (response->success) 
        {
            // RCLCPP_INFO(LOGGER, "Service call succeeded: %s", response->message.c_str());
            return true;
        } 
        else 
        {
            RCLCPP_WARN(LOGGER, "Service call failed: %s", response->message.c_str());
            return false;
        }
    } 
    else 
    {
        RCLCPP_ERROR(LOGGER, "Service call failed");
        return false;
    }
}