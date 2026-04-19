import py_trees

# Returns SUCCESS if the robot is not stuck
class NotStuck(py_trees.behaviour.Behaviour):
    def __init__(self, ros_node, name="Not stuck?"):
        super().__init__(name)
        self.ros_node = ros_node

    def update(self):
        if self.ros_node.needs_help:
            self.logger.info(f"MAARCO is calling for help.")
            return py_trees.common.Status.RUNNING
        if self.ros_node.is_stuck:
            self.logger.info(f"MAARCO detected a stuck state.")
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.SUCCESS

# Returns success if yaw error is above the given threshold
class YawErrorAbove(py_trees.behaviour.Behaviour):
    def __init__(self, ros_node, threshold, name=None):
        super().__init__(name or f"err > {threshold}?")
        self.ros_node = ros_node
        self.threshold = threshold

    def update(self):
        error = abs(self.ros_node.desired_yaw - self.ros_node.current_yaw)
        if error > self.threshold:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE

class IsWetSand(py_trees.behaviour.Behaviour):
    def __init__(self, ros_node, name="Wet sand?"):
        super().__init__(name)
        self.ros_node = ros_node

    def update(self):
        if self.ros_node.terrain == "wet_sand":
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE

class IsDrySand(py_trees.behaviour.Behaviour):
    def __init__(self, ros_node, name="Dry sand?"):
        super().__init__(name)
        self.ros_node = ros_node

    def update(self):
        if self.ros_node.terrain == "dry_sand":
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE