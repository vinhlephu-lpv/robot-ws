import abc

class PlannerInterface(abc.ABC):
    """
    Abstract interface for path planners in the ROS2 navigation stack.
    """

    @abc.abstractmethod
    def initialize(self, *args, **kwargs):
        """Initialize the planner with necessary parameters."""
        pass

    @abc.abstractmethod
    def plan(self, start_pose, goal_pose, obstacles=None, safety_margin=0.38, planning_mode="AVOIDANCE"):
        """
        Plan a path from start_pose to goal_pose avoiding obstacles.
        Returns a dictionary containing:
          - path: Path object (standardized)
          - status: Success/Failure status string
          - cost: Path cost (float)
          - duration: Time taken to plan (seconds)
          - num_nodes: Number of nodes explored
        """
        pass

    @abc.abstractmethod
    def cancel(self):
        """Cancel current planning execution."""
        pass

    @abc.abstractmethod
    def get_status(self):
        """Get the status of the planner."""
        pass

    @abc.abstractmethod
    def reset(self):
        """Reset planner state."""
        pass


class ControllerInterface(abc.ABC):
    """
    Abstract interface for vehicle controllers (tracking and path following).
    """

    @abc.abstractmethod
    def initialize(self, *args, **kwargs):
        """Initialize the controller with params."""
        pass

    @abc.abstractmethod
    def start(self):
        """Start or enable the controller."""
        pass

    @abc.abstractmethod
    def stop(self):
        """Stop or disable the controller."""
        pass

    @abc.abstractmethod
    def reset(self):
        """Reset controller state."""
        pass

    @abc.abstractmethod
    def compute_command(self, *args, **kwargs):
        """
        Compute command velocities (Twist/speeds).
        """
        pass

    @abc.abstractmethod
    def update_parameter(self, param_name, value):
        """Dynamically update controller parameter."""
        pass
