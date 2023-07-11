package main

import (
	"github.com/bluenviron/goroslib/v2/pkg/msg"
	"github.com/bluenviron/goroslib/v2/pkg/msgs/geometry_msgs"
)

type goalTaskReq struct {
	Task_id  int32
	Priority int32
	Pose     geometry_msgs.PoseStamped
}

type goalTaskRes struct {
	Success bool `json:"success"`
}

type goalTask struct {
	msg.Package `ros:"my_package"`
	goalTaskReq
	goalTaskRes
}

/////////////////////////////////////

type taskStatusReq struct {
	Task_id int32
	Status  string
}

type taskStatusRes struct {
	Success bool
}

type taskStatus struct {
	msg.Package `ros:"my_package"`
	taskStatusReq
	taskStatusRes
}
