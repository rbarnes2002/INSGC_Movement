#!/usr/bin/env python3
"""
CSV columns (ONLY these 10):
  1) robot_id
  2) robot_subtask_start_time
  3) robot_subtask_end_time
  4) human_interruption_generation_timestamp
  5) robot_deferring_human_request_ms
  6) task_type
  7) timestamp_start_attending_human_request
  8) timestamp_end_attending_human_request
  9) robot_receiving_human_request
  10) robot_total_task_start_time
  11) robot_total_task_end_time
"""

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



# Da Helpers

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

#NOTE: if this array is changed it may break onSubtaskStart, 
DEFAULT_COLUMNS = [
    "robot_id",
    "task_id", #needed for the system, can be deleted before saved to csv
    "subtask_id", #same idea as ^^^
    "robot_subtask_start_time",
    "robot_subtask_end_time",
    "human_request_timestamp",
    "robot_deferring_human_request_ms",
    "priority", #based on which type of experiment this is, one of these two columns will be deleted
    "urgency", 
    "task_type",
    "timestamp_start_attending_human_request",
    "timestamp_end_attending_human_request",
    "robot_receiving_human_request",
    "robot_total_task_start_time",
    "robot_total_task_end_time",
]

AGGREGATE_COLUMNS = [
        "robot_id",
        "distance_traveled",
        "percent_area_explored",
        "idle_time",
        "human_requests_completed",
        "human_requests_received",
        "ignored_requests",
        "failed_requests",
        "number_of_task_switches",
        "number_of_human_requests"
    ]

OPTIONAL_ID_COLUMNS = ["robot_id", "task_id", "action"]


class TaskMetricsLogger(Node):
    def __init__(
        self,
        robot_topics: List[str],
        human_topics: List[str],
        out_csv: str,
        aggr_out_csv: str,
        include_ids: bool,
        include_raw: bool,
        order_type: str,
        experiment_time: int,
        num_of_bots: int
    ):
        super().__init__("task_metrics_logger")
        
        self.out_csv = out_csv
        self.aggr_out_csv =  aggr_out_csv
        self.robot_topics = robot_topics
        self.human_topics = human_topics
        
        #main task data frame
        self.TaskDf = pd.DataFrame(columns=DEFAULT_COLUMNS)
        
        #used for aggregate data 
        self.AggrDf = pd.DataFrame(columns=AGGREGATE_COLUMNS)
        
        for botIndex in range(1, num_of_bots + 1):
            self.AggrDf.loc[len(self.AggrDf)] = [f"robot{botIndex}"] + [0.0]*(len(AGGREGATE_COLUMNS) -1)
        
        
        #keeps track of each task (not subtask)
        self.TaskRecords = []
        #this is predominantly used for robot messages that come before the human task topic Msg has been handled
        self.CallBackMsgs = []
        
        #used to keep track of task switches(key value pairs of the last task started)
        self.robotTaskDict = {}
        
        #type of experiment, either urgent or priority
        self.orderType = order_type
        
        #after this many seconds, the logger will self terminate
        self.experimentRunTime = experiment_time
        
        self.experimentTimeStart = time.time()
        
        self.numberOfHumanRequests = 0
        
        self.closed = False
        

        """
        # Ensure output directory exists
        out_dir = os.path.dirname(os.path.abspath(self.out_csv))
        if out_dir and not os.path.exists(out_dir):
            os.makedirs(out_dir, exist_ok=True)
        
        
        self.f = open(self.out_csv, "w", newline="")
        self.writer = csv.DictWriter(self.f, fieldnames=self.columns)
        self.writer.writeheader()
        self.f.flush()
        """

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
        self.get_logger().info(f"Writing CSV to: {self.out_csv} and {self.aggr_out_csv}")
        
        
       
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
            if(task_name != "collisionAvoidance"):
                self.HandleExploreMessages(jsonMsg)
           
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
                print("NOT written yet")
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
        
        #update aggregate df
        self.AggrDf.loc[self.AggrDf["robot_id"] == robot, "human_requests_received"] += 1
        self.AggrDf.to_csv(self.aggr_out_csv, index=False)
    
    #on task start update task start time in record
    def onTaskStart(self, jsonMsg):
        #update task record
        task_id = jsonMsg.get("task_id")
        TaskRec = self.findTaskRecord(task_id)
        TaskRec.task_start_unix = jsonMsg.get("ts_unix")
        
        #keep track of task switches
        self.get_logger().info("Updates task switch on task start DEBUG")
        self.handleTaskSwitches(jsonMsg)
        
    def onSubtaskStart(self, jsonMsg):
        robot = jsonMsg.get("robot")
        task_id = jsonMsg.get("task_id")
        subtask_id = jsonMsg.get("subtask_id")
        timeStamp = unix_to_iso(jsonMsg.get("ts_unix")) #subtask start time 
        
        TaskRec = self.findTaskRecord(task_id)
        
        """
        #create a new row in the df
        newRow = {"task_id": task_id, "subtask_id": subtask_id, "robot_subtask_start_time": jsonMsg.get("ts_unix")}
        #add row to the df
        self.TaskDf = pd.concat([self.TaskDf, pd.DataFrame([newRow])], ignore_index = True)
        """
        #creates a new row, the first 4 columns are filled in, the rest are blank
        self.TaskDf.loc[len(self.TaskDf["task_id"])] = [robot, task_id, subtask_id, timeStamp] + [""]*(len(DEFAULT_COLUMNS) - 4)
        
        
        
        #now fill in other known data 
        columns = ["timestamp_start_attending_human_request", "robot_total_task_start_time", "robot_deferring_human_request_ms", "priority", "urgency", "human_request_timestamp", "robot_receiving_human_request", "task_type" ]
        data = [
            unix_to_iso(TaskRec.task_start_unix),
            unix_to_iso(self.experimentTimeStart),
            str(TaskRec.task_start_unix - TaskRec.task_received_unix),
            TaskRec.priority,
            TaskRec.urgency,
            unix_to_iso(TaskRec.task_created_unix),
            unix_to_iso(TaskRec.task_received_unix),
            TaskRec.taskType
        ]
        
        self.TaskDf.loc[(self.TaskDf["task_id"] == task_id), columns ] = data
        
        #self.writeTaskData()
        self.TaskDf.to_csv(self.out_csv, index=False)
    
    def onSubtaskEnd(self, jsonMsg):
        task_id = jsonMsg.get("task_id")
        subtask_id = jsonMsg.get("subtask_id")
        
        self.TaskDf.loc[
            (self.TaskDf["task_id"] == task_id) &
            (self.TaskDf["subtask_id"] == subtask_id),
            "robot_subtask_end_time"
        ] = unix_to_iso(jsonMsg.get("ts_unix"))
        
        self.TaskDf.to_csv(self.out_csv, index=False)

    def onTaskEnd(self, jsonMsg):
        task_id = jsonMsg.get("task_id")
        robot = jsonMsg.get("robot")
        TaskRec = self.findTaskRecord(task_id)
        TaskRec.task_end_unix = jsonMsg.get("ts_unix")
        
        self.TaskDf.loc[
            self.TaskDf["task_id"] == task_id,
            "timestamp_end_attending_human_request"
        ] = unix_to_iso(jsonMsg.get("ts_unix"))
        
        self.TaskDf.to_csv(self.out_csv, index=False)
        
        #update the aggregate df 
        self.AggrDf.loc[self.AggrDf["robot_id"] == robot, "human_requests_completed"] += 1
        self.AggrDf.to_csv(self.aggr_out_csv, index=False)
            
    
    #fills in the aggregate data from the robot
    def onAggregate(self, jsonMsg):
        robot_id = jsonMsg.get("robot")
        
        #if the robot is not already in the df
        if(not (self.AggrDf["robot_id"].isin([jsonMsg.get("robot")]).any())):
            #add the row, only fills in the robot name/id
            self.AggrDf.loc[len(self.AggrDf)] = [robot_id] + [0.0]*(len(AGGREGATE_COLUMNS) -1)
            
            self.AggrDf.sort_values(by="robot_id", inplace=True)
            
        columns = [ "idle_time", "percent_area_explored", "distance_traveled"]
        data = [jsonMsg.get("idle_time"), jsonMsg.get("percent_area_explored"), jsonMsg.get("distance_traveled")]
            
        #write in the data
        self.AggrDf.loc[self.AggrDf["robot_id"] == robot_id, columns] = data
        
        self.AggrDf.to_csv(self.aggr_out_csv, index=False)
        
    def HandleExploreMessages(self, jsonMsg):
        eventType = jsonMsg.get("event")
        if(eventType == "task_start"):
            self.onExploreTaskStart(jsonMsg)
    
    def onExploreTaskStart(self, jsonMsg):
        #check/update for task switches
        #keep track of task switches
        self.get_logger().info("Updates task switch on explore start DEBUG")
        self.handleTaskSwitches(jsonMsg)
        
        
    
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
        
        self.AggrDf["number_of_human_requests"] = self.numberOfHumanRequests
        self.AggrDf.to_csv(self.aggr_out_csv, index=False)
        
        #self.writeTaskData()
        #self.TaskDf.to_csv(self.out_csv, index=False)
        
        #self.TaskDf.loc[len(self.TaskDf["task_id"]), "task_id"] = TASKID
        #self.TaskDf.to_csv(self.out_csv, index=False)
    
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
                
        print("COULDNT FIND RECORD, DEBUG")
        return None
    
    #fills in task data for the subtask rows, if used constantly is inefficient. Originally designed to only run once at the end
    #however, sometimes the rlcp fails to call .close() so this func doesnt get called
    def writeTaskData(self):
        for Task in self.TaskRecords:
            columns = ["human_request_timestamp", "priority", "urgency", "task_type"]
            data = [str(Task.task_created_unix), Task.priority, Task.urgency, Task.taskType]
            #write what is guranteed to be in taskRecord
            #self.TaskDf.loc[self.TaskDf["task_id"] == Task.task_id, ["human_request_timestamp", "robot_deferring_human_request_ms"] ] = [str(Task.task_created_unix), ]
            #write if task record has been updated
            if((Task.task_received_unix != None) and (Task.task_start_unix != None)): #if robot has recieved task
                self.TaskDf.loc[self.TaskDf["task_id"] == Task.task_id, ["robot_deferring_human_request_ms", "robot_receiving_human_request"]] = [str(Task.task_start_unix - Task.task_received_unix), str(Task.task_received_unix)]
                columns += ["robot_deferring_human_request_ms", "robot_receiving_human_request", "timestamp_start_attending_human_request"]
                data += [str(Task.task_start_unix - Task.task_received_unix), str(Task.task_received_unix), str(Task.task_start_unix)]
            
            #actually write data to df
            self.TaskDf.loc[self.TaskDf["task_id"] == Task.task_id, columns ] = data
                
            #for row in self.TaskDf.loc[self.TaskDf["task_id"] == Task.task_id]:
                #row["human_request_timestamp"] = str(Task.task_created_unix)
                ###ETC NEEDS TO BE FINISHED
                
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
                
    
    def handleTaskSwitches(self, jsonMsg):
        #keep track of task switches
        robot = jsonMsg.get("robot")
        previousTask = self.robotTaskDict.get(robot)
       
        if(jsonMsg.get("baseline_task") == "True"):
            self.robotTaskDict[robot] = "explore"
        else:
            self.robotTaskDict[robot] = "human"
            
        #if robot was not in the dict before   
        if(previousTask == None):
            previousTask = self.robotTaskDict.get(robot)
        
        #if there has been a task switch
        if(self.robotTaskDict.get(robot) != previousTask):
            self.AggrDf.loc[(self.AggrDf["robot_id"] == robot), "number_of_task_switches"] += 1
        
        
    def close(self) -> None:
        if self.closed:
            return
        self.closed = True
        
        try:
            #self.writeTaskData()
            
            #if the experiment is using a specific orderType, drop the other column
            if(self.orderType == "priority"):
                self.TaskDf.drop(columns=["urgency"], inplace=True, errors="ignore")
            elif(self.orderType == "urgency"):
                self.TaskDf.drop(columns=["priority"], inplace=True, errors="ignore")
                
            self.TaskDf["robot_total_task_end_time"] = unix_to_iso(time.time())
            
            ignored = 0
            failed = 0
            
            for task in self.TaskRecords:
                # for tasks never received by a robot
                if task.task_received_unix is None:
                    ignored += 1
                    
                # received but not completed by any robot
                elif task.task_end_unix is None:
                    failed += 1
                 
            self.AggrDf["ignored_requests"] = ignored
            self.AggrDf["failed_requests"] = failed
            
            self.TaskDf.to_csv(self.out_csv, index=False)
            self.AggrDf.to_csv(self.aggr_out_csv, index=False)
            print("CLOSED CORRECTLY")
        # in case something goes wrong while saving CSVs
        except Exception as e:
            print(f"ERROR while closing logger: {e}")
            
            
def default_filename() -> str:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return os.path.expanduser(f"~/Desktop/task_metrics_{stamp}.csv")
    
def aggr_default_filename() -> str:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return os.path.expanduser(f"~/Desktop/task_metrics_aggregate_{stamp}.csv")
    
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
    p.add_argument("--aggr-out", default= aggr_default_filename())
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
        aggr_out_csv=args.aggr_out,
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
