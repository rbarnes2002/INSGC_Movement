#!/usr/bin/env python3
# task_metrics_logger_V4.py
# written by Ryan Barnes
import argparse
import csv
import json
import os
import time
from dataclasses import dataclass
from datetime import datetime
from typing import Dict, Optional, Set, List, Tuple

import pandas as pd

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.executors import ExternalShutdownException

import Tasks

# helpers
def unix_to_iso(ts_unix: float) -> str:
    return datetime.fromtimestamp(ts_unix).strftime("%H:%M:%S")
    
@dataclass
class HumanInterrupt:
    task_id: str
    created_unix: float
    urgency: str = ""
    priority: str = ""
    is_priority: Optional[bool] = None
    action: str = ""
    robot_hint: str = ""


@dataclass
class TaskRecord:
    robot_id: str = ""
    task_id: str = ""

    # robot event timestamps
    subtask_start_unix: Optional[float] = None
    subtask_end_unix: Optional[float] = None
    task_received_unix: Optional[float] = None
    task_start_unix: Optional[float] = None
    task_end_unix: Optional[float] = None
    
    task_created_unix: Optional[float] = None

    # fields to classify task
    urgency: Optional[int] = None
    priority: Optional[int] = None
    
    taskType: Optional[String] = None
    
    is_priority: Optional[bool] = None
    baseline_task: Optional[bool] = None

    # bookkeeping
    is_human_task: bool = False
    finalized: bool = False
    
DEFAULT_COLUMNS = [
    "robot_id",
    "task_id", #needed for the system, can be deleted before saved to csv
    "subtask_id", #same idea as ^^^
    "robot_subtask_start_time",
    "robot_subtask_end_time",
    "human_request_timestamp",
    "robot_deferring_human_request_s",
    "priority", #based on which type of experiment this is, one of these two columns will be deleted
    "urgency", 
    "task_type",
    "timestamp_start_attending_human_request",
    "timestamp_end_attending_human_request",
    "robot_receiving_human_request",
    "robot_total_task_start_time",
    "robot_total_task_end_time",
    "explore_task_start_time", # this is to measure when the actual explore task started
    "explore_task_end_time", # and this is to measure when it ends, so we can assess idle time
    "distance_traveled",
    "percent_area_explored",
    "idle_time",
    "human_requests_completed",
    "human_requests_received",
    "ignored_requests",
    "failed_requests",
    "number_of_task_switches",
    "number_of_human_requests",
    "acceptance_percent",
    "completion_percent"
]

class TaskMetricsLogger(Node):
    def __init__(
        self,
        robot_topics: List[str],
        human_topics: List[str],
        out_csv: str,
        include_ids: bool,
        include_raw: bool,
        order_type: str,
        experiment_time: int,
        num_of_bots: int
    ):
        super().__init__("task_metrics_logger")
        
        self.out_csv = out_csv
        self.robot_topics = robot_topics
        self.human_topics = human_topics
        
        #main task data frame
        self.TaskDf = pd.DataFrame(columns=DEFAULT_COLUMNS)
        
        #keeps track of each task (not subtask)
        self.TaskRecords = []
        #this is predominantly used for robot messages that come before the human task topic Msg has been handled
        self.CallBackMsgs = []
        
        #used to keep track of task switches(key value pairs of the last task started)
        self.robotTaskDict = {}
        
        # Per-robot aggregate statistics
        self.robotStats = {}
        
        for i in range(1, num_of_bots + 1):
            robot = f"robot{i}"
            
            self.robotStats[robot] = {
                "distance_traveled": 0.0,
                "percent_area_explored": 0.0,
                "idle_time": 0.0,
                "human_requests_completed": 0,
                "human_requests_received": 0,
                "number_of_task_switches": 0,
            }
            
        
        #type of experiment, either urgent or priority
        self.orderType = order_type
        
        #after this many seconds, the logger will self terminate
        self.experimentRunTime = experiment_time
        
        self.experimentTimeStart = time.time()
        
        self.numberOfHumanRequests = 0
        
        self.closed = False
        
        # Subscriptions
        self.robot_subs = []
        for t in self.robot_topics:
            self.robot_subs.append(self.create_subscription(String, t, self.on_robot_event, 200))

        self.human_subs = []
        for t in self.human_topics:
            self.human_subs.append(self.create_subscription(String, t, self.on_human_task, 200))
        
        self.timer_subscription = None
        #if a experiment run time has been chosen
        if(self.experimentRunTime > 0):
            self.timer_subscription = self.create_timer(5, self.checkRunTime)

        self.get_logger().info(f"Robot event topics: {', '.join(self.robot_topics)}")
        self.get_logger().info(f"Human task topics: {', '.join(self.human_topics)}")
        self.get_logger().info(f"Writing CSV to: {self.out_csv}")

    #it is assumed that logMsg is a json string
    def on_robot_event(self, logMsg: String):
        #find which case the msg is
        eventType: String = None
        task_id: String = None
        jsonMsg: String = None
        
        self.get_logger().info(logMsg.data)
        
        try:
            jsonMsg = json.loads(logMsg.data)
        except Exception:
            print("invalid message")
            
        try:
            #get event type
            eventType = jsonMsg.get("event")
            task_id = jsonMsg.get("task_id")
            task_name = jsonMsg.get("task_name")
            int(task_id) # if this fails then that means that the id is Explore_robotX, collision etc
        except Exception:
            task_id = None
            print(task_id)

        ### for messages that do NOT have a task id
        #if the message does not have a task id, then it is an aggregate log message
        if(eventType == "aggregate"): 
            self.onAggregate(jsonMsg)
        elif(jsonMsg.get("baseline_task") == "True"):#handles explore messages
            # Ignore collision avoidance
            if task_name == "collisionAvoidance":
                return
            if eventType == "task_start":
                self.onExploreTaskStart(jsonMsg)
            elif eventType == "task_end":
                self.onExploreTaskEnd(jsonMsg)
            return
           
        #makes sure that on_human_task was called first, if not then store the message for later writing
        elif((task_id != None) and (self.findTaskRecord(jsonMsg.get("task_id")) != None)):
            if(eventType == "task_received"):
                self.onTaskRec(jsonMsg)
            elif(eventType == "subtask_start"):
                self.onSubtaskStart(jsonMsg)
            elif(eventType == "subtask_end"):
                self.onSubtaskEnd(jsonMsg)
            elif(eventType == "task_start"):
                self.onTaskStart(jsonMsg)
            elif(eventType == "task_end"):
                self.onTaskEnd(jsonMsg)
            elif(eventType == "task_interrupted"):
                robot = jsonMsg.get("robot")
                self.robotStats[robot]["number_of_task_switches"] += 1
                print(f"DEBUG: task switch recorded for {robot}")
                
        else:
            self.CallBackMsgs.append(logMsg)

    #adds new row to the csv
    def onTaskRec(self, jsonMsg):
        robot = jsonMsg.get("robot")
        task_id = jsonMsg.get("task_id")
        subtask_id = jsonMsg.get("subtask_id")
                
        #task record of the current task
        TaskRec = self.findTaskRecord(task_id)
        
        #modify task record 
        TaskRec.task_received_unix = jsonMsg.get("ts_unix")
        TaskRec.robot_id = robot
        
        self.robotStats[robot]["human_requests_received"] += 1

    #on task start update task start time in record
    def onTaskStart(self, jsonMsg):
        #update task record
        task_id = jsonMsg.get("task_id")
        TaskRec = self.findTaskRecord(task_id)
        TaskRec.task_start_unix = jsonMsg.get("ts_unix")
        
        #keep track of task switches
        self.get_logger().info("Updates task switch on task start DEBUG")
        
    def onSubtaskStart(self, jsonMsg):
        robot = jsonMsg.get("robot")
        task_id = jsonMsg.get("task_id")
        subtask_id = jsonMsg.get("subtask_id")
        timeStamp = unix_to_iso(jsonMsg.get("ts_unix")) #subtask start time 
        
        TaskRec = self.findTaskRecord(task_id)
        
        # Ignore duplicate subtask_start events
        if (
            (
                (self.TaskDf["task_id"] == task_id)
                & (self.TaskDf["subtask_id"] == subtask_id)
            ).any()
        ):
            return
            
        #creates a new row, the first 4 columns are filled in, the rest are blank
        self.TaskDf.loc[len(self.TaskDf["task_id"])] = [robot, task_id, subtask_id, timeStamp] + [""]*(len(DEFAULT_COLUMNS) - 4)
        
        
        
        #now fill in other known data 
        columns = ["timestamp_start_attending_human_request", "robot_total_task_start_time", "robot_deferring_human_request_s", "priority", "urgency", "human_request_timestamp", "robot_receiving_human_request", "task_type" ]
        data = [
            unix_to_iso(TaskRec.task_start_unix),
            unix_to_iso(self.experimentTimeStart),
            str(TaskRec.task_start_unix - TaskRec.task_received_unix),
            TaskRec.priority,
            TaskRec.urgency,
            unix_to_iso(TaskRec.task_created_unix),
            robot,
            TaskRec.taskType
        ]
        
        self.TaskDf.loc[(self.TaskDf["task_id"] == task_id), columns ] = data
        
        #self.writeTaskData()
        # self.TaskDf.to_csv(self.out_csv, index=False)
    
    def onSubtaskEnd(self, jsonMsg):
        task_id = jsonMsg.get("task_id")
        subtask_id = jsonMsg.get("subtask_id")
        
        self.TaskDf.loc[
            (self.TaskDf["task_id"] == task_id) &
            (self.TaskDf["subtask_id"] == subtask_id),
            "robot_subtask_end_time"
        ] = unix_to_iso(jsonMsg.get("ts_unix"))
        
        # self.TaskDf.to_csv(self.out_csv, index=False)

    def onTaskEnd(self, jsonMsg):
        task_id = jsonMsg.get("task_id")
        robot = jsonMsg.get("robot")
        TaskRec = self.findTaskRecord(task_id)
        TaskRec.task_end_unix = jsonMsg.get("ts_unix")
        
        self.TaskDf.loc[
            self.TaskDf["task_id"] == task_id,
            "timestamp_end_attending_human_request"
        ] = unix_to_iso(jsonMsg.get("ts_unix"))
        
        # self.TaskDf.to_csv(self.out_csv, index=False)
        
        self.robotStats[robot]["human_requests_completed"] += 1
        
    def onAggregate(self, jsonMsg):

        robot = jsonMsg.get("robot")

        if robot not in self.robotStats:
            return

        stats = self.robotStats[robot]

        if jsonMsg.get("distance_traveled") is not None:
            stats["distance_traveled"] = jsonMsg["distance_traveled"]

        if jsonMsg.get("idle_time") is not None:
            stats["idle_time"] = jsonMsg["idle_time"]

        if jsonMsg.get("percent_area_explored") is not None:
            stats["percent_area_explored"] = jsonMsg["percent_area_explored"]
            
    def onExploreTaskStart(self, jsonMsg):
        robot = jsonMsg.get("robot")
        task_id = jsonMsg.get("task_id")
        
        # Only create one row for this exploration task
        if (
            (self.TaskDf["task_id"] == task_id)
            & (self.TaskDf["robot_id"] == robot)
        ).any():
            return
            
        row = {col: "" for col in DEFAULT_COLUMNS}
        row["robot_id"] = robot
        row["task_id"] = task_id
        row["task_type"] = jsonMsg.get("task_name")
        row["subtask_id"] = ""
        row["priority"] = ""
        row["urgency"] = ""
        row["explore_task_start_time"] = unix_to_iso(jsonMsg.get("ts_unix"))
        
        self.TaskDf.loc[len(self.TaskDf)] = row
            
    def onExploreTaskEnd(self, jsonMsg):
        robot = jsonMsg.get("robot")
        task_id = jsonMsg.get("task_id")
        
        self.TaskDf.loc[
            (self.TaskDf["robot_id"] == robot)
            &
            (self.TaskDf["task_id"] == task_id),
            "explore_task_end_time"
        ] = unix_to_iso(jsonMsg.get("ts_unix"))
    
    #this is mainly for adding new tasks to the task records list
    def on_human_task(self, userCmd):
        print(userCmd.data)
        #print(f"DEBUG: on_human_task() called for task {Tasks.ParseMsg(userCmd.data)[5]}")

        TYPE, FROM, TO, URGENCY, PRIORITY, TASKID, TASK, PARAMS = Tasks.ParseMsg(userCmd.data)
        
        # ignore duplicate callbacks
        if self.findTaskRecord(TASKID) is not None:
            print(f"DEBUG: Duplicate human task {TASKID} ignored")
            return
            
        #self.TaskDf[len(self.TaskDf["task_id"])] = {"task_id" : TASKID, "human_request_timestamp" : time.time()}
        TempRec = TaskRecord()
        TempRec.task_id = TASKID
        TempRec.is_human_task = True
        TempRec.task_created_unix = time.time()
        TempRec.priority = PRIORITY
        TempRec.urgency = URGENCY
        TempRec.taskType = TASK
        self.TaskRecords.append(TempRec)
        
        self.handleCallBackMsgs()
        
        self.numberOfHumanRequests +=1 
    
    #this helps handle robot messages for some task that come before the on_human_task call
    def handleCallBackMsgs(self):
        maxIndex = len(self.CallBackMsgs)
        
        for msgIndex in range(0, maxIndex):
            print("There has been a callback DEBUG")
            print(self.CallBackMsgs[msgIndex].data)
            self.on_robot_event(self.CallBackMsgs[msgIndex])
            
        for msgIndex in range(0, maxIndex):
            self.CallBackMsgs.pop(0)

        
    def findTaskRecord(self, taskID) -> TaskRecord:
        for rec in self.TaskRecords:
            if(rec.task_id == taskID):
                return rec
                
        return None
    
    def checkRunTime(self):
        if(self.experimentRunTime > 0):
            if((time.time() - self.experimentTimeStart) > self.experimentRunTime):
                print("EXPERIMENT ENDED, OUT OF RUN TIME, DEBUG")
                
                # stop the timer
                if self.timer_subscription is not None:
                    self.timer_subscription.cancel()
                
                self.close()
                
                # so it does not hang in terminal when closing
                if rclpy.ok():
                    rclpy.shutdown()
                
            #write in the robot_total_task_end_time, this is for rundancy in case the logger crashes (we will have a rough end time), this value should be written in at the end
            self.TaskDf["robot_total_task_end_time"] = unix_to_iso(time.time())
            self.TaskDf.to_csv(self.out_csv, index=False)
                
    def close(self) -> None:
    
        if self.closed:
            return
        self.closed = True
        
        # if the experiment is using a specific orderType, drop the other column
        if self.orderType == "priority":
            self.TaskDf.drop(columns=["urgency"], inplace=True, errors="ignore")
        elif self.orderType == "urgency":
            self.TaskDf.drop(columns=["priority"], inplace=True, errors="ignore")

        # Fill in experiment end time
        self.TaskDf["robot_total_task_end_time"] = unix_to_iso(time.time())

        # Count ignored and failed requests
        ignored = 0
        failed = 0

        for task in self.TaskRecords:
            if task.task_received_unix is None:
                ignored += 1
            elif task.task_end_unix is None:
                failed += 1

        # Calculate acceptance/completion percentages
        accepted = 0
        completed = 0

        for stats in self.robotStats.values():
            accepted += stats["human_requests_received"]
            completed += stats["human_requests_completed"]

        if self.numberOfHumanRequests > 0:
            acceptance = accepted / self.numberOfHumanRequests * 100.0
            completion = completed / self.numberOfHumanRequests * 100.0
        else:
            acceptance = 0.0
            completion = 0.0

        # Copy robot statistics into every task row
        for idx in self.TaskDf.index:

            robot = self.TaskDf.at[idx, "robot_id"]

            if robot not in self.robotStats:
                continue

            stats = self.robotStats[robot]

            self.TaskDf.at[idx, "distance_traveled"] = stats["distance_traveled"]
            self.TaskDf.at[idx, "percent_area_explored"] = stats["percent_area_explored"]
            self.TaskDf.at[idx, "idle_time"] = stats["idle_time"]
            self.TaskDf.at[idx, "human_requests_completed"] = stats["human_requests_completed"]
            self.TaskDf.at[idx, "human_requests_received"] = stats["human_requests_received"]
            self.TaskDf.at[idx, "number_of_task_switches"] = stats["number_of_task_switches"]

        # Experiment-wide values (same for every row)
        self.TaskDf["ignored_requests"] = ignored
        self.TaskDf["failed_requests"] = failed
        self.TaskDf["number_of_human_requests"] = self.numberOfHumanRequests
        self.TaskDf["acceptance_percent"] = acceptance
        self.TaskDf["completion_percent"] = completion
        self.TaskDf.fillna("", inplace=True)

        # Save one CSV
        self.TaskDf.to_csv(self.out_csv, index=False)

        print("CLOSED CORRECTLY")
            
def default_filename() -> str:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return os.path.expanduser(f"~/Desktop/task_metrics_{stamp}.csv")
    
def main():
    p = argparse.ArgumentParser()
    p.add_argument(
        "--robot-topic",
        action="append",
        default=[],
        help="Robot event topic(s) (std_msgs/String JSON). Repeatable. Default: /task_events",
    )
    p.add_argument(
        "--human-topic",
        action="append",
        default=[],
        help="Human task topic(s). Can be JSON (/robot1/human_task) and/or ForRyan keyboard publisher (userTopic). Repeatable.",
    )
    p.add_argument("--out", default=default_filename())
    p.add_argument("--include-ids", action="store_true", help="Add robot_id/task_id/action columns (in addition to the 10 categories).")
    p.add_argument("--include-raw", action="store_true", help="Add raw_event_json column for debugging.")
    p.add_argument("--task-order-type", action="store", default="both", help="Determines whether priority, urgency, or both are stored")
    p.add_argument("--experiment-run-time", action="store", default=300, type=int, help="How long the experiment is, the logger will self-terminate to ensure that data is written to the CSVs. Note: enter 0 to turn off this feature")
    p.add_argument("--numOfBots", type = int, default=5, help="Number of bots that the user can send interruptions to")
    args = p.parse_args()

    robot_topics = args.robot_topic if args.robot_topic else ["/task_events"]
    # Default to both, so you don't have to remember which pipeline you're using.
    human_topics = args.human_topic if args.human_topic else ["/robot1/human_task", "userTopic"]

    rclpy.init()
    node = TaskMetricsLogger(
        robot_topics=robot_topics,
        human_topics=human_topics,
        out_csv=args.out,
        include_ids=args.include_ids,
        include_raw=args.include_raw,
        order_type=args.task_order_type,
        experiment_time=args.experiment_run_time,
        num_of_bots = args.numOfBots
    )
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
