# Required: Pose2D specifications of the trajectory waypoints.
geometry_msgs/Pose2D[] waypoints
# Optional: Motion models to use for each segment.
cobot_msgs/MotionModel2D[] motion_models
# Required: The bounding motion model for the entire trajectory.
cobot_msgs/MotionModel2D motion_model_bounds
