task_metrics_logger_updated.py README
This readme is specifically just for this python file.
Written by me (unfortunately) Ryan Barnes

The logger is a ROS2 node that listens to two message streams:

1) Robot event stream
	a) Topic example: /task_events or /robot1/task_events
	b) Publishes events such as:
	c) task_received
	d) subtask_start
	e) subtask_end
	f) task_start
	g) task_end

2) Human interruption stream
	a) Topic example: /robot1/human_task
	b) Contains the timestamp when a human-generated task was created

The node correlates these two streams and produces one CSV row per interruption task containing the 10 metrics Dr. Roy had specified.

HOW IT WORKS!!!!!!

Step 1) It Starts a ROS2 Node
The program initializes a ROS2 node and creates two subscribers:
robot_topic  -> robot events
human_topic  -> human interruption events

Step 2) Store the Events Temporarily
The logger maintains in-memory dictionaries to track tasks as they progress.

Here is an example structure:
tasks = {
   task_id : {
       human_created_time
       robot_subtask_start
       robot_subtask_end
       robot_task_start
       robot_task_end
       robot_received_time
   }
}
Events update this structure until enough information exists to log the task.

Step 3) When the human task arrives
When a human interruption message is received:
created_unix
task_type
robot_id
The logger:
1) Creates a new task entry
2) Stores the human interruption timestamp
3) Stores task type

Step 4) Robot Events Arrive

Robot event messages update the same task entry.

Here is an example:
| Event         | What gets recorded                 |
| ------------- | ---------------------------------- |
| task_received | robot responding time              |
| subtask_start | robot subtask start                |
| subtask_end   | robot subtask end                  |
| task_start    | robot begins handling interruption |
| task_end      | robot finished                     |


Step 5) Compute Derived Metrics
Some columns are not directly published, so the logger calculates them.
Deferral:
deferral_ms = (task_start_time - human_created_time) * 1000
This measures how long the robot waited before handling the human request.

Robot Total Task Duration:
total_start = first_robot_event
total_end   = last_robot_event

This captures the total time spent on the robot’s work cycle.

Step 6) Write to the CSV Row
Once a task has sufficient information (usually when task_end occurs), the logger writes:

robot_subtask_start_time
robot_subtask_end_time
human_interruption_generation_timestamp
robot_deferring_human_request_ms
task_type
timestamp_start_attending_human_request
timestamp_end_attending_human_request
robot_responding_to_human_request
robot_total_task_start_time
robot_total_task_end_time

Each row corresponds to one human interruption task.
