package main

import (
	"fmt"
"net/http"
	"github.com/gin-gonic/gin"
	"github.com/bluenviron/goroslib/v2"
	"github.com/bluenviron/goroslib/v2/pkg/msg"
	"github.com/bluenviron/goroslib/v2/pkg/msgs/geometry_msgs"
)

type goalTaskReq struct {
	Task_id   int32
	Priority int32
	Pose     geometry_msgs.PoseStamped
}

type goalTaskRes struct {
	Success bool
}

type goalTask struct {
	msg.Package `ros:"my_package"`
	goalTaskReq
	goalTaskRes
}

var node *goroslib.Node

func main() {
	r := gin.Default()

	r.POST("/api/tasks", handleTasks)

	go startROSNode()

	if err := r.Run(":8080"); err != nil {
		panic(err)
	}
}

func handleTasks(c *gin.Context) {
	var req goalTaskReq
	if err := c.ShouldBindJSON(&req); err != nil {
		c.JSON(http.StatusBadRequest, gin.H{"error": err.Error()})
		return
	}

	// Send the goal to ROS service and get the response
	res := goalTaskRes{}
	err := sendGoal(&req, &res)
	if err != nil {
		c.JSON(http.StatusInternalServerError, gin.H{"error": err.Error()})
		return
	}

	c.JSON(http.StatusOK, res)
}

func sendGoal(req *goalTaskReq, res *goalTaskRes) error {
	if node == nil {
		return fmt.Errorf("ROS node is not initialized")
	}

	sc, err := goroslib.NewServiceClient(goroslib.ServiceClientConf{
		Node: node,
		Name: "assign_goal",
		Srv:  &goalTask{},
	})
	if err != nil {
		return err
	}
	defer sc.Close()

	

	srvRes := &goalTaskRes{}
	err = sc.Call(req, srvRes)
	if err != nil {
		return err
	}

	res.Success = srvRes.Success
	return nil
}

func startROSNode() {
	var err error
	node, err = goroslib.NewNode(goroslib.NodeConf{
		Name:          "ros_node",
		MasterAddress: "127.0.0.1:11311",
	})
	if err != nil {
		panic(err)
	}
	defer node.Close()

	
}

