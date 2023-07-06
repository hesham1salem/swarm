package main

import (
	"bytes"
	"fmt"
	"net/http"

	"github.com/bluenviron/goroslib/v2"
	"github.com/bluenviron/goroslib/v2/pkg/msg"
	"github.com/bluenviron/goroslib/v2/pkg/msgs/geometry_msgs"
	"github.com/gin-gonic/gin"
)

type goalTaskReq struct {
	Task_id  int32
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
	// send string to io.Reader

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
	fmt.Println(req)
	// Send the goal to ROS service and get the response
	res := goalTaskRes{}

	/* err := sendGoal(&req, &res)
	if err != nil {
		c.JSON(http.StatusInternalServerError, gin.H{"error": err.Error()})
		return
	} */
	//	sendRequest()

	c.JSON(http.StatusOK, res)

	sendRequest()

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

func sendRequest() {
	url := "http://192.168.1.9:8000/api/tasks/14"
	payload := []byte(`{
		"status": "failure"
	}`)

	req, err := http.NewRequest("POST", url, bytes.NewBuffer(payload))
	if err != nil {
		fmt.Println("Error creating request:", err)
		return
	}

	req.Header.Set("Content-Type", "application/json")
	req.Header.Set("Accept", "application/json")

	client := &http.Client{}
	resp, err := client.Do(req)
	if err != nil {
		fmt.Println("Error sending request:", err)
		return
	}
	defer resp.Body.Close()

	fmt.Println("Response Status:", resp.Status)
	// Process the response as needed
}
