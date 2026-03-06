task_metrics_logger_updated.py README
This readme is specifically just for this python file.
Written by Ryan Barnes 

The logger is a ROS2 node that listens to two message streams and
records robot reaction metrics when a human interruption task occurs.

The node correlates robot execution events with human-generated interruptions
and writes a single CSV row per interruption task containing the 10 metrics that were defined by Dr. Roy.

MESSAGE STREAMS

Robot Event Stream

Example topics:
/task_events
/robot1/task_events

These events describe the robot’s internal task execution timeline.

Typical events include:

task_received
Robot acknowledges an incoming task

subtask_start
Robot begins a navigation or exploration subtask

subtask_end
Robot finishes that subtask

task_start
Robot begins handling the human interruption

task_end
Robot finishes handling the interruption

These events are used to determine robot reaction time and task duration.

Human Interruption Stream

Example topics:
/robot1/human_task
userTopic

Human tasks may be published in two formats depending on which controller is running.

JSON message format

Contains fields such as:
created_unix
task_type
robot_id

String command format (Lorence's system)

Example command:

interruption server all <urgency> <priority> <task_id>

This message indicates that a human interruption task has been generated.

The logger extracts:
task creation timestamp
urgency flag
priority flag
task id

TASK TYPE DETERMINATION

The logger determines the task type using the urgency and priority flags.

Rules used by the system:

priority != 2 → task_type = priority

priority == 2 AND urgency == 1 -> task_type = urgent

priority == 2 AND urgency == 0 -> task_type = non_urgent

This rule is necessary because the system uses priority = 2 for normal urgent/non-urgent tasks.

HOW IT WORKS

Step 1) Start the ROS2 Node

The program initializes a ROS2 node and creates two subscribers.

robot_topic -> robot execution events
human_topic -> human interruption events

Step 2) Store Events Temporarily

The logger maintains in-memory dictionaries to track tasks while they are still running.

Example structure:

tasks = {
task_id : {
human_request_time
robot_subtask_start
robot_subtask_end
robot_task_start
robot_task_end
robot_received_time
}
}

Events update this structure until enough information exists to log the task.

Step 3) When a Human Task Arrives

When a human interruption message is received the logger:

Creates a new task entry

Stores the human interruption timestamp

Determines and stores the task type

Stores the task id

Step 4) Robot Events Arrive

Robot event messages update the corresponding task entry.

Event → What gets recorded

task_received → robot responding time
subtask_start → robot subtask start
subtask_end → robot subtask end
task_start → robot begins attending interruption
task_end → robot finishes interruption

Step 5) Compute Derived Metrics

Some columns are not directly published and must be calculated.

Robot Deferral Time

deferral_ms = (task_start_time - human_created_time) * 1000

This measures how long the robot waited before attending to the human request.

Robot Total Task Duration

total_start = first_robot_event
total_end = last_robot_event

This captures the total duration of the robot’s work cycle including interruptions.

Step 6) Write a CSV Row

Once enough information exists (usually when task_end occurs), the logger writes a row containing the 10 metrics:

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

OUTPUT

The node writes all metrics to a CSV file.

Each row represents one interruption event and the robot’s response timeline.
